# head_front_camera ↔ Velodyne calibration — session notes

Runbook for calibrating the TIAGo Pro's **head_front_camera** (RealSense) against the
**Velodyne VLP-32C**, distilled from the first end-to-end session. Covers the head-camera
variant specifically — see `README.md` for the generic D455/chest-camera version and the
underlying method (ChArUco plane correspondence + SVD).

Config: `config/calibration_head_front.yaml`
Result: `~/.ros/lidar_camera_calibration/lidar_to_head_front_camera.yaml`

---

## Key facts learned this session

- **Real camera optical frame is `head_front_camera_color_optical_frame`.** The live
  `camera_info.header.frame_id` on `/head_front_camera/color/camera_info` reports
  `rgbd_camera_color_optical_frame` — that frame is a driver-internal name and is **not**
  connected to anything in the robot's TF tree. Always pass
  `--camera-frame head_front_camera_color_optical_frame` to `estimate_transform`.
- **Run collection/validation ON the robot, not the dev machine.** Raw LiDAR + camera
  topics are too much for wifi. Only `/calibration/debug_image` needs to reach the dev
  machine for the visual check, and even that should be compressed first (see below).
- **You need a genuinely open space.** A 13m² room was not enough — the ROI's far edge
  (x_max: 4.0m in LiDAR frame ⇒ ~3.6m from camera) puts a wall directly behind/near the
  board for most poses, and RANSAC silently locks onto that wall instead of the board.
  Symptom: `dist_lidar` (see diagnostic below) stays nearly frozen across samples that
  should have different distances. Use a corridor or larger room with ≥5m clear depth.
- **Board too close is just as bad as a wall behind it.** The ROI's `x_min: 1.9` is in the
  LiDAR frame; since the camera sits ~0.375m in front of the LiDAR, the board must be held
  **≥1.8m from the camera** (not eyeballed — mark distances on the floor with tape) or its
  points fall outside the ROI entirely and RANSAC fits background instead.
- **Rotation needs roll variation, not just yaw/pitch.** Plane-normal-only SVD cannot
  fully constrain rotation about the board's own normal axis unless some captures roll
  the board diagonally (like a diamond), not just tilt it left/right/up/down. Symptom:
  low per-sample residuals but the debug-image projection shows straight edges as visibly
  **rotated** rather than just offset. Fixed by adding ~10+ samples with deliberate roll
  variation — angular error dropped from ~8° (std ~13°, garbage) to ~1.4° (std ~0.7°,
  tight and consistent) once roll was included.
- **A single-digit or double-digit-degree outlier sample (e.g. 40–60°) shows up almost
  every batch.** It's a real detection failure (partial/misread ChArUco), not a fitting
  artifact — always drop anything >5° before trusting the fit.
- **This calibration is only valid at the exact head pan/tilt pose it was captured at.**
  The head is on a movable joint; a `static_transform_publisher` from
  `head_front_camera_color_optical_frame → velodyne` is wrong the instant the head moves.
  **Not yet solved** — see Outstanding work below.

---

## Bugs fixed in the package this session

- `collect_samples_node.py` / `validate_projection_node.py`: image + camera_info
  subscribers used default RELIABLE QoS, but the RealSense driver publishes BEST_EFFORT.
  Result: subscriber silently received nothing (`No valid detection` forever). Fixed by
  passing `qos_profile=qos_profile_sensor_data` on those two subscribers in both nodes.
  (LiDAR subscriber was already RELIABLE↔RELIABLE, left unchanged.)
- `collect_samples_node.py` / `validate_projection_node.py` / `publish_transform.launch.py`:
  `output_file`/`result_file` params used `~` but nothing called `.expanduser()`, so paths
  like `~/.ros/...` either crashed (`PermissionError` trying to create `/home/<wrong-user>`)
  or failed silently. Fixed with `.expanduser()` at each read site.
- `calibration_head_front.yaml` originally hardcoded `/home/cas/...` (dev machine) paths —
  broke immediately on the robot (`pal` user). Changed to `~/...` now that expanduser works.

---

## Fast path for next time

```bash
# 1. On the robot: sync + build
cd /home/pal/outdoor_robot_ws/src/outdoor_social_robot && git pull
cd /home/pal/outdoor_robot_ws && colcon build --packages-select lidar_camera_calibration
source install/setup.bash

# 2. Position robot somewhere with ≥5m clear depth, ≥1.2m clear width. Mark floor
#    distances at 2m / 2.75m / 3.5m from the camera with tape.

# 3. Collect (sensors already running on robot)
ros2 launch lidar_camera_calibration collect.launch.py \
    launch_lidar:=false launch_camera:=false \
    config_file:=/home/pal/outdoor_robot_ws/src/outdoor_social_robot/bringup_devices/lidar_camera_calibration/config/calibration_head_front.yaml

# capture: hold board steady ~5-7s at each pose (auto-captures every 5s), OR force:
ros2 service call /calibration/capture std_srvs/srv/Trigger
```

Vary **all** of: distance (1.8–3.5m from camera), lateral position, pitch/yaw tilt, AND
roll (diamond/diagonal orientation) — roughly even coverage across ~15-20 poses.

**Check quality as you go** (every 4-5 captures), don't wait until the end:

```bash
python3 -c "
import json, numpy as np
with open('/home/pal/.ros/lidar_camera_calibration/samples_head_front.json') as f:
    data = json.load(f)
for i, s in enumerate(data['samples'], start=1):
    c_l = np.array(s['c_lidar']); c_c = np.array(s['c_cam'])
    dist_lidar = np.linalg.norm(c_l); dist_cam = c_c[2]
    print(f'{i:3d}  dist_lidar={dist_lidar:6.3f}m  dist_cam={dist_cam:6.3f}m  diff={dist_lidar-dist_cam:+6.3f}m')
"
```
Healthy: `diff` in the ~0.5–0.9m range (varies from sample to sample — it should track
`dist_cam`, not sit frozen). Frozen `dist_lidar` while `dist_cam` moves = contaminated
sample, discard it (its index) before estimating.

```bash
# 4. Estimate (drop any index flagged >5° angular residual, or found bad by the
#    diagnostic above, via a quick python filter — see conversation history for the
#    exact filter-script pattern)
ros2 run lidar_camera_calibration estimate_transform \
    --samples ~/.ros/lidar_camera_calibration/samples_head_front.json \
    --output  ~/.ros/lidar_camera_calibration/lidar_to_head_front_camera.yaml \
    --camera-frame head_front_camera_color_optical_frame

# 5. Validate — node runs on robot, only debug image needs to cross wifi (compressed)
ros2 launch lidar_camera_calibration validate.launch.py \
    launch_lidar:=false launch_camera:=false \
    config_file:=/home/pal/outdoor_robot_ws/src/outdoor_social_robot/bringup_devices/lidar_camera_calibration/config/calibration_head_front.yaml \
    result_file:=/home/pal/.ros/lidar_camera_calibration/lidar_to_head_front_camera.yaml

ros2 run image_transport republish raw compressed \
    --ros-args -r in:=/calibration/debug_image -r out/compressed:=/calibration/debug_image/compressed
# dev machine: rqt_image_view → /calibration/debug_image/compressed
```

Check edges at both near (~1.5-2m) and far (~3m+) range. Look for: offset (translation
error → recollect with better distance discipline) vs. rotated lines (roll error →
recollect with more roll variation).

---

## Outstanding work: dynamic transform for the movable head

Not solved this session — punted with a static YAML for now, valid only at the head pose
used during this calibration.

The right fix is **not** a runtime republishing node. `robot_state_publisher` already
publishes a live TF chain through the head's pan/tilt joints from `/joint_states`; what's
wrong is presumably a small fixed mounting offset somewhere in that chain (the joint
between the head tilt link and the camera, or the camera's internal `_link →
_color_optical_frame` offset). The plan:

1. Find the head/camera URDF/xacro — **not vendored in this dev workspace** (checked;
   likely lives on the robot under an installed PAL description package, similar to how
   `communication_skills` PAL actions are on-robot-only — see memory). Locate it with:
   `find / -iname "*.xacro" 2>/dev/null | xargs grep -l "head_front_camera" 2>/dev/null`
2. If it's a package we actually control (in `outdoor_social_robot` or similar), compare
   our calibrated `velodyne → head_front_camera_color_optical_frame` transform (at the
   exact head pose used for calibration) against what that URDF chain currently predicts
   at the same joint state, and correct the fixed offset to match.
3. If it's a vendor-installed PAL package (likely, given it wasn't found in-tree), edits
   would be overwritten on update — need a different strategy (e.g. an overlay xacro, or
   a correction node that reads `/joint_states` and applies our fixed offset via forward
   kinematics). Not yet decided.
