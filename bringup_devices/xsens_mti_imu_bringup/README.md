# Xsens MTi-M320 IMU Bringup

ROS 2 bringup for the **Xsens MTi-320-8A7G6** (AHRS — 9-DOF: accelerometer, gyroscope, magnetometer).  
Depends on `bluespace_ai_xsens_mti_driver`, which is included in the workspace and compiles the XDA library from source.

---

## Published topics

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/filter/quaternion` | `geometry_msgs/QuaternionStamped` | 100 Hz | Orientation from on-board Kalman filter |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | Orientation quaternion (angular vel / accel fields are invalid — use separate topics) |
| `/imu/angular_velocity` | `geometry_msgs/Vector3Stamped` | 100 Hz | Gyroscope (rad/s) |
| `/imu/acceleration` | `geometry_msgs/Vector3Stamped` | 100 Hz | Linear acceleration with gravity (m/s²) |
| `/filter/free_acceleration` | `geometry_msgs/Vector3Stamped` | 100 Hz | Gravity-compensated acceleration (m/s²) |
| `/imu/mag` | `sensor_msgs/MagneticField` | 100 Hz | Magnetometer (a.u.) |
| `/imu/dq` | `geometry_msgs/QuaternionStamped` | 100 Hz | Orientation increments |
| `/imu/dv` | `geometry_msgs/Vector3Stamped` | 100 Hz | Velocity increments |
| `/imu/time_ref` | `sensor_msgs/TimeReference` | 100 Hz | Device sample timestamp |
| `/temperature` | `sensor_msgs/Temperature` | 1 Hz | Sensor chip temperature (°C) |

Frame id: `imu_link`

---

## PC — build and launch

### 1. Build (first time only)

```bash
cd ~/outdoor_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bluespace_ai_xsens_mti_driver xsens_mti_imu_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> The `bluespace_ai_xsens_mti_driver` build step compiles the bundled XDA C++ library (~30 s).  
> Serial port permissions: your user must be in the `dialout` group (`sudo usermod -aG dialout $USER`, then log out and back in).

### 2. Plug in the IMU

The device enumerates as `/dev/ttyUSB0` via the `xsens_mt` USB-serial kernel module (already present on Ubuntu 22.04).

### 3. Launch

```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 launch xsens_mti_imu_bringup xsens_mti_imu_bringup.launch.py
```

Expected startup output:
```
[xsens_mti_node] Scanning for devices...
[xsens_mti_node] Found a device with ID: 02D023BA @ port: /dev/ttyUSB0, baudrate: 115200
[xsens_mti_node] Device: MTi-320-8A7G6, with ID: 02D023BA opened.
[xsens_mti_node] Output configuration set: quat+accel+gyro+mag+temp
[xsens_mti_node] Measuring ...
```

### 4. Verify

```bash
# Check topics are live
ros2 topic list | grep -E "imu|filter|temp"

# Sanity-check acceleration (Z ≈ 9.81 m/s² when flat)
ros2 topic echo /imu/acceleration --once

# Check rate (~90–100 Hz)
ros2 topic hz /imu/angular_velocity
```

### 5. Custom config (optional)

Edit `config/xsens_m320.yaml` to change rates or enable/disable topics, then rebuild and relaunch:

```bash
colcon build --packages-select xsens_mti_imu_bringup
ros2 launch xsens_mti_imu_bringup xsens_mti_imu_bringup.launch.py \
    config_file:=/path/to/my_params.yaml
```

---

## Jetson Orin — deploy and launch

### 1. Copy packages to the Jetson

From the PC, sync both driver and bringup packages:

```bash
JETSON_IP=<jetson-ip>
JETSON_USER=<user>

rsync -av ~/outdoor_robot_ws/src/bluespace_ai_xsens_ros_mti_driver \
          ~/outdoor_robot_ws/src/bringup_devices/xsens_mti_imu_bringup \
    ${JETSON_USER}@${JETSON_IP}:~/outdoor_robot_ws/src/
```

> If the Jetson does not yet have an `outdoor_robot_ws/src/` directory, create it first:  
> `ssh ${JETSON_USER}@${JETSON_IP} "mkdir -p ~/outdoor_robot_ws/src"`

### 2. Install build dependencies on the Jetson

```bash
sudo apt update
sudo apt install -y \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    build-essential cmake
```

### 3. Build on the Jetson

```bash
cd ~/outdoor_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select bluespace_ai_xsens_mti_driver xsens_mti_imu_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4. Serial port permissions on the Jetson

```bash
# Add your user to dialout (one-time)
sudo usermod -aG dialout $USER
# Apply without reboot
newgrp dialout
```

Verify the device appears after plugging in:

```bash
ls /dev/ttyUSB*   # should show /dev/ttyUSB0
```

If the port does not appear, load the kernel module manually:

```bash
sudo modprobe xsens_mt
```

### 5. Launch on the Jetson

```bash
source ~/outdoor_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=1   # must match the rest of the robot network
ros2 launch xsens_mti_imu_bringup xsens_mti_imu_bringup.launch.py
```

### 6. Read data from the PC

With `ROS_DOMAIN_ID=1` set on both machines and both on the same network:

```bash
# On the PC
export ROS_DOMAIN_ID=1
ros2 topic echo /imu/angular_velocity --once
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `No MTi device found` | Port already held by another process | `lsof /dev/ttyUSB0` → kill the holder |
| `No MTi device found` | Permission denied on port | Add user to `dialout`, re-login |
| Port does not appear | `xsens_mt` module not loaded | `sudo modprobe xsens_mt` |
| Only `/filter/quaternion` publishes | Old firmware output config | Restart node — `setOutputConfiguration` is called on every startup |
| Rate is ~90 Hz instead of 100 Hz | USB/ROS scheduling jitter | Normal; configure Cyclone DDS or reduce other USB traffic if precision matters |
