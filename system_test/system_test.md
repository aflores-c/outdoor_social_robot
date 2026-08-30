# Full system test — runbook

Everything needed to bring the whole school-traffic-control system up across
the four machines involved, from a fresh map through a live run. Verified
against the actual launch-file source in this repo (exact args below), but
**not live-tested end-to-end in this pass** — the dev machine's network link
was down while this was written. Validate each machine individually the
first time you run it.

## Machines

| Machine | IP | User | Role |
|---|---|---|---|
| Robot onboard PC | 10.68.0.1 | pal | Localization, navigation, traffic control state machine |
| Dev computer | 10.68.0.209 | (yours) | RViz only |
| Jetson (perception) | 10.68.0.206 | pal | traffic_object_detection + vehicle_plate_detection_fastalpr |
| Jetson (drone) | 10.68.0.208 | tiago-jetson | drone_traffic_perception + GPS/IMU (data storage only) |

Passwords aren't recorded here — see the team's own credentials reference
for these machines.

Scripts referenced below live alongside this file, in `src/system_test/` in
this repo: `run_robot.sh`, `run_jetson_perception_206.sh`,
`run_jetson_drone_208.sh`. Copy each onto its target machine (e.g. `scp
src/system_test/run_robot.sh pal@10.68.0.1:~/`) and run it there — or `ssh
<user>@<ip> 'bash -s' < src/system_test/run_X.sh` from the dev machine.
Since this whole file lives under `src/`, it (and the scripts) travel with
the rest of the repo whenever you sync/deploy it to the robot or a Jetson —
no separate copy step needed for the docs themselves.

Every script sets `ROS_DOMAIN_ID=2` and logs each node to
`~/system_test_logs/<name>.log` (`tail -f` any of them to check status).

---

## Phase 0 — one-time per-machine setup (skip if already done)

Not scripted — these are the environment/dependency prerequisites already
documented per-package:

- Jetson 10.68.0.206: `perception/traffic_object_detection/DEPLOYMENT.md`
  (venv `~/venvs/yolo_ros`) and
  `perception/vehicle_plate_detection_fastalpr/DEPLOYMENT.md` (venv
  `~/venvs/plate_detection_fastalpr`).
- Jetson 10.68.0.208: `perception/drone_traffic_perception/README.md` (venv
  `~/visdrone_deployment/venv`).
- Robot 10.68.0.1: workspace already built at
  `~/outdoor_robot_ws` (`/home/pal/outdoor_robot_ws/src/outdoor_social_robot/...`).

If you've changed any of the code touched this session (school_traffic_control,
base_scan_proximity, robot_audio, benchmark_logging, traffic_perception_msgs,
drone_traffic_perception), rebuild + resource on whichever machine(s) it
changed before running the scripts:
```bash
cd ~/outdoor_robot_ws   # or ~/ros2_ws on the Jetsons
colcon build --packages-select <changed packages>
source install/setup.bash
```

---

## Phase 1 — build a map (one-time per site, robot only)

This is interactive (you drive the robot and judge coverage), so it's not
scripted. Run on the robot (10.68.0.1):

```bash
export ROS_DOMAIN_ID=2
source /opt/pal/alum/setup.bash
source ~/outdoor_robot_ws/install/setup.bash

ros2 launch velodyne_vlp32c_bringup vlp32c_outdoor.launch.py &
sleep 3
ros2 launch scan_matcher_bringup scan_matcher.launch.py &
sleep 2
ros2 launch amcl_2d_localization mapping.launch.py &
```

Drive the robot around the whole area you'll test in (the crossing, pose A
and pose B locations, the parking-redirect route if relevant). Watch `/map`
in RViz2 on the dev computer as you go — cells fill in as it explores.

When coverage looks complete, from another terminal on the robot:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
This writes `~/map.pgm` + `~/map.yaml`. Copy them into the package's actual
map folder on **this robot's real checkout path** (note: this differs from
the generic path in `localization_2d.md` — that doc assumes a flat
`~/outdoor_robot_ws/src/...` layout; this robot nests the repo one level
deeper under `outdoor_social_robot/`):
```bash
cp ~/map.pgm ~/map.yaml \
    ~/outdoor_robot_ws/src/outdoor_social_robot/localization/amcl_2d_localization/map/

cd ~/outdoor_robot_ws
colcon build --packages-select amcl_2d_localization
```
Saving directly as `map.pgm`/`map.yaml` (not a custom name) means
`amcl_localization.launch.py` picks it up with no extra args later — that's
what `run_robot.sh` assumes. If you want to keep multiple maps around, save
under a different name and pass `map_name:=<name>` to that launch file
instead (see `localization_2d.md` for the full option list).

Stop the three mapping processes (`Ctrl+C` each, or `pkill -f
"vlp32c_outdoor\|scan_matcher\|mapping.launch"`) before moving to Phase 2 —
`mapping.launch.py`'s slam_toolbox and `amcl_localization.launch.py`'s AMCL
both publish `map → odom` and will fight each other if both are up.

---

## Phase 2 — full system run

### 2.1 Robot (10.68.0.1)

```bash
scp src/system_test/run_robot.sh pal@10.68.0.1:~/
ssh pal@10.68.0.1 './run_robot.sh'
```
Brings up: velodyne → scan_matcher → AMCL (using the map from Phase 1) →
base_navigation → PAL's own base-laser bringup + base_scan_proximity →
robot_audio → school_traffic_control → benchmark_logging. Comment out the
base-laser/base_scan_proximity block or the benchmark_logging line inside
the script first if you don't need either for this particular test.

### 2.2 RViz (10.68.0.209, dev computer)

Just RViz2 — add displays for `/map`, `/amcl_pose`
(PoseWithCovariance), `/particle_cloud`, `/scan_outdoor` (LaserScan). Use the
**2D Pose Estimate** tool to give AMCL a rough starting pose matching the
robot's real physical location, then drive/teleop the robot a little until
the particle cloud converges (confirm with `ros2 topic hz /particle_cloud`
and `ros2 run tf2_ros tf2_echo map base_link` on the robot — should track
smoothly, not jump).

### 2.3 Capture pose A and pose B (first time at a new site only)

Once localization has converged, drive the robot to the "middle of the
road" holding spot and read its pose:
```bash
ros2 run tf2_ros tf2_echo map base_link
```
Take the `x`, `y`, and yaw-in-degrees from the RPY (degree) line — that's
pose A. Drive to the pulled-aside spot and repeat for pose B. Fill both into
`decision_making/school_traffic_control/config/school_traffic_control.yaml`
(`pose_a_x/y/phi_deg`, `pose_b_x/y/phi_deg`), then on the robot:
```bash
cd ~/outdoor_robot_ws
colcon build --packages-select school_traffic_control
```
and restart the `school_traffic_control` process from `run_robot.sh`'s log
directory (kill its PID, re-run just that one `ros2 launch` line, or re-run
the whole script — the earlier nodes will just log "already running"-style
warnings from a second AMCL/velodyne instance, so prefer killing and
restarting only that one node in practice).

### 2.4 Jetson — perception (10.68.0.206)

```bash
scp src/system_test/run_jetson_perception_206.sh pal@10.68.0.206:~/
ssh pal@10.68.0.206 './run_jetson_perception_206.sh'
```
Brings up `traffic_object_detection` (head_front_camera) and
`vehicle_plate_detection_fastalpr`. Both stay idle (perception-load
switching) until `school_traffic_control` enables them as a vehicle
actually approaches — that's expected, not a bug.

### 2.5 Jetson — drone (10.68.0.208)

This script relies on `~/.bashrc` already exporting the correct
`RMW_IMPLEMENTATION`/`CYCLONEDDS_URI` for this machine (with the right DDS
peers to reach the robot) — run it from an **interactive** login, not a
one-line `ssh host 'command'` invocation, since a non-interactive SSH
command doesn't reliably source `.bashrc` and will silently fall back to
the wrong RMW:
```bash
scp src/system_test/run_jetson_drone_208.sh tiago-jetson@10.68.0.208:~/
ssh tiago-jetson@10.68.0.208     # log in interactively
./run_jetson_drone_208.sh
```
Brings up GPS + IMU (data storage only, not consumed by the state machine
yet) and the drone/VisDrone perception over RTMP. Verify with:
```bash
ros2 topic echo /drone_vehicle_detections
ros2 topic echo /drone_vehicle_detections_link_status
```

---

## Phase 3 — verify the whole chain

From any machine on the domain:
```bash
ros2 topic hz /amcl_pose                          # localization alive
ros2 topic echo /perception/vehicles               # camera+lidar vehicle detections
ros2 topic echo /perception/plate_result            # plate reads (once a vehicle stops)
ros2 topic echo /perception/close_proximity          # base-scan safety net
ros2 topic echo /drone_vehicle_detections_link_status # drone link status
```
Watch `school_traffic_control`'s own log (`~/system_test_logs/school_traffic_control.log`
on the robot) for state transitions as a test vehicle approaches.

If running `benchmark_logging`, start/stop a trial to confirm data lands
correctly (see the package's own docstrings, or ask for the earlier
walkthrough of `start_trial`/`stop_trial`).

---

## Debugging perception

Topics to check per perception node when something isn't showing up in
`school_traffic_control`. A topic appearing in `ros2 topic list` only means
it's been declared — it doesn't mean data is flowing; check `ros2 topic hz`
or `echo` to confirm messages are actually arriving. Both detection nodes
default to their `enabled_topic` being off at rest — `school_traffic_control`
toggles them; force it manually (see the Forcing signals table below) to
test either node standalone.

**`traffic_object_detection`** (jetson 10.68.0.206, `yolo_ros` venv):

| Topic | Type | Notes |
|---|---|---|
| `/perception/traffic_object_detection_enabled` | `std_msgs/Bool` | Gate — node stays idle until `true` |
| `/head_front_camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | Input image |
| `/head_front_camera/color/camera_info` | `sensor_msgs/CameraInfo` | Input |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Input, for 3D pose extraction |
| TF `head_front_camera_color_optical_frame` → `velodyne` | — | Read live every frame; a broken TF chain silently kills pose extraction. Check with `ros2 run tf2_ros tf2_echo velodyne head_front_camera_color_optical_frame` |
| `/perception/vehicles` | `traffic_perception_msgs/msg/VehicleDetectionArray` | Output, consumed by `school_traffic_control` |
| `/perception/pedestrians` | `traffic_perception_msgs/msg/PedestrianDetectionArray` | Output, consumed by `school_traffic_control` |
| `/traffic_object_detection/debug_image` | `sensor_msgs/Image` | Annotated boxes + LiDAR overlay (`rqt_image_view`), throttled to `debug_fps` |
| `/traffic_object_detection/vehicles/poses`, `/pedestrians/poses` | `geometry_msgs/PoseArray` | 3D poses, viewable in RViz |
| `/traffic_object_detection/vehicles/markers`, `/pedestrians/markers` | `visualization_msgs/MarkerArray` | RViz markers |

If `debug_image` shows correct boxes but `/perception/vehicles`/`pedestrians`
stay empty, the issue is downstream — the LiDAR-point-count gate
(`*_min_lidar_points`) or the TF lookup, not YOLO itself.

**`vehicle_plate_detection_fastalpr`** (jetson 10.68.0.206, `plate_detection_fastalpr` venv):

| Topic | Type | Notes |
|---|---|---|
| `/perception/plate_detection_enabled` | `std_msgs/Bool` (TRANSIENT_LOCAL) | Gate — node stays idle until `true` |
| `/head_front_camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | Input image |
| `/perception/plate_allowed` | `std_msgs/Bool` | Output — true if any visible plate matches the allow-list; consumed by `school_traffic_control` |
| `/perception/plate_result` | `traffic_perception_msgs/msg/PlateResult` | Output — one message per detected plate box (`plate_text`, `det_confidence`, `ocr_confidence`, `authorized`) |
| `/vehicle_plate_detection_fastalpr/last_plate` | `std_msgs/String` | Most recent OCR'd plate text |
| `/vehicle_plate_detection_fastalpr/debug_image` | `sensor_msgs/Image` | Annotated debug image (`rqt_image_view`), throttled to `debug_fps` |

If `plate_allowed` never goes `true`, check `config/registered_plates.yaml`
isn't empty — the node logs a startup warning and rejects every plate if so.

Sanity sequence for either node:
```bash
ros2 topic pub --once <enabled_topic> std_msgs/msg/Bool "{data: true}"
ros2 topic hz <output_topic>
ros2 run rqt_image_view rqt_image_view   # pick the debug_image topic
```

---

## Forcing signals — driving the state machine manually

For bench/dry testing without live perception hardware running, every
input `school_traffic_control_node` reads is a plain topic — publish to any
of these to force a transition. Topic names below are the current defaults
from `decision_making/school_traffic_control/config/school_traffic_control.yaml`
— override there if you've renamed anything.

**Freshness matters**: `vehicles_topic`, `pedestrians_topic`,
`plate_allowed_topic`, and `close_proximity_topic` are all subject to
`message_timeout_s` (default 1.0s) — a single one-shot publish goes stale
within a second and the node falls back to "nothing detected". Use `ros2
topic pub -r 10 ...` (matching `control_rate_hz`) to hold a value, not
`-1`. `force_plate_allowed_topic` and `emergency_topic` have no freshness
check (the last value received wins, indefinitely) — but a single `-1`
publish can still be lost to a DDS discovery race if the subscriber hasn't
matched yet (hit this once this session); prefer a short repeated burst
(`-r 10` for ~1s, then Ctrl+C) over a bare `-1` for those too.

| Signal | Topic | Type | Forces |
|---|---|---|---|
| Vehicle in range | `/perception/vehicles` | `traffic_perception_msgs/msg/VehicleDetectionArray` | MIDDLE_IDLE → VEHICLE_STOP, for any vehicle with `range_near_m ≤ distance ≤ range_far_m` (default 5-10m) |
| Vehicle stopped | same topic, `stopped: true` on the closest in-range vehicle | — | VEHICLE_STOP → CHECK_PLATE |
| Second vehicle queued | same array, 2 entries with **distinct `id`** both in range | — | Sets `_had_second_vehicle` (changes RETURNING's gesture to `motion_stop` instead of `motion_arms_init`) — **currently dormant on the real system**: the live perception stack always publishes `id: -1` (no persistent tracking), so this only exercises via a hand-crafted test array, not real hardware |
| Plate vote | `/perception/plate_allowed` | `std_msgs/Bool` | CHECK_PLATE → CROSS_VEHICLE once ≥ `plate_vote_min_yes` (2) of the last `plate_vote_window` (5) readings are `true` |
| Force-authorize plate | `/perception/force_plate_allowed` | `std_msgs/Bool` | CHECK_PLATE → CROSS_VEHICLE immediately, bypassing the vote — **one-shot**, auto-clears after authorizing a single vehicle |
| Pedestrian detected | `/perception/pedestrians` | `traffic_perception_msgs/msg/PedestrianDetectionArray` | Plays `pedestrian_introduction_message` (if currently MIDDLE_IDLE) or `pedestrian_alert_message` (any other state) when within `pedestrian_alert_range_m` (10m — matches vehicles' `range_far_m`, pedestrians have no near bound) — audio only, never changes motion/state |
| Close proximity (base scan) | `/perception/close_proximity` | `std_msgs/Bool` | Same pedestrian-alert effect as above, OR'd with the camera/lidar check |
| Drone parking counts | `drone_vehicle_detections` | `drone_traffic_perception/msg/VehicleDetectionCounts` | Chooses `go_to_sfo_audio.mp3` vs `stop_audio.mp3` in WAIT_TO_LEAVE — statistical mode of `raw_detections` over the trailing `parking_count_window_s` (1s), compared against `parking_free_threshold` (12) |
| Emergency | `/school_traffic_control/emergency` | `std_msgs/Bool` | Any state → EMERGENCY (cancels in-flight nav/motion goals, holds for teleop); `false` → back to MIDDLE_IDLE |
| Switch to plate perception | `/perception/plate_detection_enabled` | `std_msgs/Bool` (TRANSIENT_LOCAL) | Turns `vehicle_plate_detection_fastalpr` on/off directly — `school_traffic_control` normally drives this itself (on only during CHECK_PLATE), force it to test the plate node standalone without going through the state machine |
| Switch to car/pedestrian perception | `/perception/traffic_object_detection_enabled` | `std_msgs/Bool` (TRANSIENT_LOCAL) | Turns `traffic_object_detection` on/off directly — same idea, for testing vehicle/pedestrian detection standalone. `school_traffic_control` keeps this on through VEHICLE_STOP and only swaps to plate perception during CHECK_PLATE, so the two are normally mutually exclusive |

### Example commands

Force a vehicle into range:
```bash
ros2 topic pub -r 10 /perception/vehicles traffic_perception_msgs/msg/VehicleDetectionArray \
  "{vehicles: [{id: 1, distance: 5.0, stopped: false}]}"
```
Mark it stopped (same pattern, `stopped: true`):
```bash
ros2 topic pub -r 10 /perception/vehicles traffic_perception_msgs/msg/VehicleDetectionArray \
  "{vehicles: [{id: 1, distance: 5.0, stopped: true}]}"
```
Force-authorize the plate (skips CHECK_PLATE's vote/timeout):
```bash
ros2 topic pub -r 10 /perception/force_plate_allowed std_msgs/msg/Bool "{data: true}"
```
Switch perception modes by hand (bypasses `school_traffic_control`'s own
switching, for testing either detection node standalone):
```bash
ros2 topic pub --once /perception/plate_detection_enabled std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /perception/traffic_object_detection_enabled std_msgs/msg/Bool "{data: false}"
```
Trigger EMERGENCY, then clear it once you've confirmed `state=EMERGENCY` in
the log:
```bash
ros2 topic pub -r 10 /school_traffic_control/emergency std_msgs/msg/Bool "{data: true}"
# Ctrl+C once confirmed, then:
ros2 topic pub -r 10 /school_traffic_control/emergency std_msgs/msg/Bool "{data: false}"
```
Report a free parking spot outside the school (feeds WAIT_TO_LEAVE's audio choice):
```bash
ros2 topic pub -r 10 drone_vehicle_detections drone_traffic_perception/msg/VehicleDetectionCounts \
  "{raw_detections: 5, ema_detections: 5, average_detections: 5}"
```

Stop any freshness-gated publisher (`Ctrl+C`) once you've confirmed the
transition — the state machine treats it going stale as "vehicle/pedestrian
left", which is itself how you force the *next* transition (e.g.
VEHICLE_STOP → MIDDLE_IDLE, or CHECK_VEHICLE_IN_RANGE → RETURNING).

Watch `school_traffic_control_node`'s own log — every trigger above also
publishes a matching JSON event on `/benchmark/events`
(`ros2 topic echo /benchmark/events`) if you want machine-readable
confirmation instead of parsing log lines.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| AMCL never converges | Bad initial pose | Re-set 2D Pose Estimate closer to the real location |
| `map_server` fails to load | Map not copied into `map/`, or not rebuilt | Redo the copy + `colcon build` step in Phase 1 |
| `tf2_echo map base_link` fails | AMCL/lifecycle_manager not active yet | Check `run_robot.sh`'s `amcl.log` for lifecycle transition errors |
| Plate/vehicle detection never turns on | `school_traffic_control` never sees a vehicle in range | Check `range_near_m`/`range_far_m` in its config vs. actual test distance |
| `/scan` empty, base_scan_proximity silent | PAL's `omni_base_laser_sensors` bringup not running | It's not part of this git workspace — start it explicitly (see `run_robot.sh`'s comment) |
| Drone topics never appear | RTMP link down, or wrong venv active | Check `drone_traffic_perception.log`; confirm `USE_RTMP`/`RTMP_URL` in `main.py` |
| Nothing discovers across machines | `ROS_DOMAIN_ID` mismatch, or CycloneDDS `<Peers>`/`<NetworkInterface>` misconfigured for a machine's active interface | Confirm `echo $ROS_DOMAIN_ID` is `2` everywhere; check `cyclonedds.xml` peers include this machine |
