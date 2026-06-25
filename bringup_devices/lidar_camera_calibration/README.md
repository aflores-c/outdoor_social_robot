# lidar_camera_calibration

Extrinsic calibration between the **Velodyne VLP-32C** and the **Intel RealSense D455**.

Estimates and publishes the 6-DOF rigid transform:

```
camera_color_optical_frame  →  velodyne
```

---

## Method: ChArUco plane correspondence + SVD

A large ChArUco board acts as the calibration target.  For each board pose:

- **Camera side**: OpenCV detects ChArUco corners → `solvePnP` → board plane (normal + centroid) in `camera_color_optical_frame`
- **LiDAR side**: RANSAC on the ROI-filtered point cloud → board plane (normal + centroid) in `velodyne` frame

With N ≥ 6 plane pairs, SVD (Kabsch algorithm) solves for rotation R and least-squares solves for translation t:

```
R @ n_lidar_i  ≈  n_cam_i       (normals must be parallel after rotation)
t  =  mean_i(c_cam_i - R @ c_lidar_i)   (centroids must coincide after transform)
```

**Why ChArUco over other targets:**
- Sub-pixel accurate corner detection (better than plain ArUco)
- Works with partial occlusion — each corner has a unique ID
- Full native OpenCV 4.x support; no extra libraries needed
- The board plane is robust enough for sparse LiDAR (VLP-32C sees 100–200 returns on a 70×50 cm board at 3 m)

---

## 0. Prerequisites

```bash
# Already installed in this workspace:
#   ros-humble-realsense2-camera, ros-humble-librealsense2
#   ros-humble-velodyne-*  (via velodyne_vlp32c_bringup)
#   python3-opencv, python3-numpy, python3-scipy

# Build the package:
cd ~/outdoor_robot_ws
colcon build --packages-select lidar_camera_calibration
source install/setup.bash
```

---

## 1. Print and mount the ChArUco board

### 1.1 Generate the board image

```bash
python3 - <<'EOF'
import cv2, cv2.aruco as a
d = a.Dictionary_get(a.DICT_4X4_50)
b = a.CharucoBoard_create(5, 7, 0.10, 0.075, d)   # 5×7, 10 cm squares, 7.5 cm markers
img = b.draw((1000, 1400))
cv2.imwrite('charuco_board.png', img)
print('Saved: charuco_board.png')
EOF
```

### 1.2 Print and measure

- Print on **A1 paper** (594 × 841 mm) or larger — the board image is 50 × 70 cm at 10 cm/square
- Laminate or glue to a **rigid foam board or aluminium sheet** — it must be perfectly flat
- After printing, measure the actual square size with a ruler and update `config/calibration.yaml`:

```yaml
board:
  square_size_m: 0.100    # ← measure and adjust
  marker_size_m: 0.075    # ← measure and adjust (≈ 75% of square_size)
```

### 1.3 Mount the sensors

The calibration is more accurate if both sensors are in their final mounted positions on the robot.  The board is moved — the sensors stay fixed.

---

## 2. Tune the LiDAR ROI

Before collecting, narrow the LiDAR region-of-interest to the area where you will hold the board.  This prevents RANSAC from accidentally fitting the ground or a wall.

Edit `config/calibration.yaml`:

```yaml
lidar_roi:
  x_min:  0.5    # metres in front of velodyne frame
  x_max:  6.0
  y_min: -1.5
  y_max:  1.5
  z_min:  0.2    # raise if ground appears inside ROI
  z_max:  2.0
```

Quick check — visualise the ROI while the LiDAR is running:

```bash
ros2 launch velodyne_vlp32c_bringup vlp32c.launch.py
# In RViz2: add PointCloud2 → /velodyne_points, set Fixed Frame=velodyne
# Check that your board area is inside the box defined by the ROI limits above
```

---

## 3. Collect samples

```bash
# Terminal 1 — start everything
ros2 launch lidar_camera_calibration collect.launch.py

# Terminal 2 — monitor the debug image
ros2 run rqt_image_view rqt_image_view
# → select topic: /calibration/debug_image
```

The debug image shows:
- `BOARD: OK (N corners)` — ChArUco detected; board axes drawn
- `LIDAR: OK` — plane fitted in point cloud

### 3.1 Capture samples

Hold the board still, wait for both detections to show OK, then:

```bash
# Terminal 3 — capture each sample
ros2 service call /calibration/capture std_srvs/srv/Trigger
```

**Between each sample, change the board pose significantly:**
- Tilt left / tilt right (vary roll)
- Tilt up / tilt down (vary pitch)
- Rotate in-plane (vary yaw)
- Move closer / farther (2 m to 5 m)

Avoid two consecutive samples with nearly identical orientations — the SVD needs diverse normal vectors to constrain all three rotation axes.

**Recommended: ≥ 8 samples with diverse orientations.**

Samples are auto-saved after every capture to:
```
~/.ros/lidar_camera_calibration/samples.json
```

### 3.2 Verify sample count and stop

```bash
# Check how many samples are saved
cat ~/.ros/lidar_camera_calibration/samples.json | python3 -c \
    "import json,sys; d=json.load(sys.stdin); print(len(d['samples']), 'samples')"

# Stop collection (Ctrl+C in Terminal 1)
```

### 3.3 Alternative: rosbag replay

If you recorded a bag during a session:

```bash
ros2 launch lidar_camera_calibration collect.launch.py \
    launch_lidar:=false  launch_camera:=false

# In another terminal:
ros2 bag play <your_bag.db3>
# Then capture samples from the bag the same way
```

---

## 4. Estimate the transform

```bash
ros2 run lidar_camera_calibration estimate_transform
```

With a custom samples path:

```bash
ros2 run lidar_camera_calibration estimate_transform \
    --samples ~/.ros/lidar_camera_calibration/samples.json \
    --output  ~/.ros/lidar_camera_calibration/lidar_to_camera.yaml
```

### Example output

```
════════════════════════════════════════════════════════════════
 LIDAR → CAMERA EXTRINSIC CALIBRATION RESULT
════════════════════════════════════════════════════════════════

Transform:  velodyne  →  camera_color_optical_frame

Rotation matrix R:
  [+0.999832  +0.001234  -0.018234]
  [-0.001112  +0.999985  +0.005432]
  [+0.018240  -0.005410  +0.999819]

Translation t [metres]:
  x = -0.12345678
  y = +0.04567890
  z = -0.28765432

Quaternion (x, y, z, w):
  x=-0.00271234  y=-0.00912345  z=+0.00061234  w=+0.99995312

Euler angles XYZ [degrees]:
  roll=+0.3105°  pitch=-1.0432°  yaw=+0.0702°

Per-sample residuals:
   #    angular [°]   translation [mm]  note
  ---  ------------  ----------------  ----
    1         0.312             8.2
    2         0.441            11.3
    3         0.623             9.8
    ...

  Mean angular error  : 0.423°  (std 0.121°)
  Mean translation err: 10.1 mm
  (good calibration: mean angular < 1°,  translation < 20 mm)

════════════════════════════════════════════════════════════════
 static_transform_publisher command
════════════════════════════════════════════════════════════════
  ros2 run tf2_ros static_transform_publisher \
    --x -0.12345678 --y 0.04567890 --z -0.28765432 \
    --qx -0.00271234 --qy -0.00912345 --qz 0.00061234 --qw 0.99995312 \
    --frame-id camera_color_optical_frame --child-frame-id velodyne
```

**Quality thresholds:**

| Metric | Good | Acceptable | Re-calibrate |
|--------|------|------------|--------------|
| Mean angular error | < 0.5° | 0.5–1.5° | > 1.5° |
| Mean translation error | < 10 mm | 10–30 mm | > 30 mm |

If individual samples show > 3° angular error, they are outliers — remove them from `samples.json` (edit the JSON manually) and re-run `estimate_transform`.

---

## 5. Validate the result

```bash
ros2 launch lidar_camera_calibration validate.launch.py

# In another terminal:
ros2 run rqt_image_view rqt_image_view
# → select /calibration/debug_image
```

**What to look for:**

| What you see | Meaning |
|---|---|
| LiDAR dots land exactly on physical edges and surfaces | Calibration is good |
| Dots are shifted in a consistent direction | Translation error — re-collect |
| Dots are rotated (tilted away from edges) | Rotation error — need more diverse board orientations |
| Dots are sparse on the board | ROI may be too narrow; board too far or too close |

Hold the calibration board in the camera+LiDAR FOV.  The board edges in the image should be lined up with the boundary of the LiDAR dot cluster.

---

## 6. Deploy the calibrated transform

### Option A — launch file (recommended)

Add to your robot bringup:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

lidar_camera_tf = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('lidar_camera_calibration'),
            'launch', 'publish_transform.launch.py',
        ])
    ]),
)
```

Or run standalone:

```bash
ros2 launch lidar_camera_calibration publish_transform.launch.py
# Optional: point to a specific file
ros2 launch lidar_camera_calibration publish_transform.launch.py \
    result_file:=/path/to/lidar_to_camera.yaml
```

### Option B — direct static_transform_publisher

Copy the command printed by `estimate_transform` and paste it into your bringup launch file as a `Node`:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_camera_tf',
    arguments=[
        '--x', '-0.12345678',
        '--y',  '0.04567890',
        '--z', '-0.28765432',
        '--qx', '-0.00271234',
        '--qy', '-0.00912345',
        '--qz',  '0.00061234',
        '--qw',  '0.99995312',
        '--frame-id',       'camera_color_optical_frame',
        '--child-frame-id', 'velodyne',
    ],
)
```

---

## 7. File locations

| File | Description |
|---|---|
| `config/calibration.yaml` | Board dimensions, ROI, RANSAC params |
| `launch/collect.launch.py` | Start sensors + sample collector |
| `launch/publish_transform.launch.py` | Publish calibrated TF from YAML |
| `launch/validate.launch.py` | Project LiDAR onto image for visual check |
| `lidar_camera_calibration/collect_samples_node.py` | ROS2 node: detection + RANSAC + service |
| `lidar_camera_calibration/estimate_transform.py` | Offline SVD solver |
| `lidar_camera_calibration/validate_projection_node.py` | Real-time projection validator |
| `~/.ros/lidar_camera_calibration/samples.json` | Collected plane samples (runtime output) |
| `~/.ros/lidar_camera_calibration/lidar_to_camera.yaml` | Calibration result (runtime output) |

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `BOARD: NOT DETECTED` | Poor lighting or board too far | Move closer (2–4 m), improve lighting |
| `BOARD: NOT DETECTED` | Square/marker size mismatch | Measure board and update `calibration.yaml` |
| `LIDAR: PLANE NOT FOUND` | ROI too tight | Widen `lidar_roi` in `calibration.yaml` |
| `LIDAR: PLANE NOT FOUND` | Fewer than `min_inliers` hits | Board is too small or too far; lower `min_inliers` |
| High angular error (> 3°) | Outlier sample | Remove the sample from `samples.json`; re-run estimate |
| Dots shifted consistently in image | Translation error | Collect more samples at varied distances |
| Dots rotated from edges | Rotation error | Ensure board tilts cover all 3 rotation axes |
| `FileNotFoundError: lidar_to_camera.yaml` | estimate not run yet | Run `estimate_transform` first |
| Sync never fires (no callbacks) | Topic names mismatch | Echo topics; adjust `image_topic`/`lidar_topic` in config |
