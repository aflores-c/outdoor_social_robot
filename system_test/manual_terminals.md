# Manual bringup — one terminal per package

Same bringup as `run_robot.sh` / `run_jetson_drone_208.sh` /
`run_jetson_perception_206.sh`, but as standalone commands instead of a
script — open a separate terminal (or `ssh` session) per command below so
each package can be checked individually. Order within each section
matches the scripts' launch order (some packages depend on an earlier one
already being up), but nothing here backgrounds/disowns anything — closing
a terminal kills that terminal's node, which is the point.

Assumes the same prerequisites as `system_test.md`: a map already built and
saved, `pose_a_x/y/phi_deg`/`pose_b_x/y/phi_deg` already filled in for an
existing site, and `sparkfun_rtk_gps_bringup`'s
`config/ntrip_credentials.yaml` already filled in.

---

## Run robot (10.68.0.1, user `pal`)

```bash
ssh pal@10.68.0.1
```
then in that shell for every command below:
```bash
export ROS_DOMAIN_ID=2
source /opt/pal/alum/setup.bash
source ~/outdoor_robot_ws/install/setup.bash
```

**1. GPS (RTK/NTRIP, SAPOS BW)**
```bash
ros2 launch sparkfun_rtk_gps_bringup gps_rtk.launch.py
```

**2. Velodyne**
```bash
ros2 launch velodyne_vlp32c_bringup vlp32c_outdoor.launch.py
```

**3. Scan matcher** (needs velodyne up first)
```bash
ros2 launch scan_matcher_bringup scan_matcher.launch.py
```

**4. AMCL** (needs scan_matcher up first)
```bash
ros2 launch amcl_2d_localization amcl_localization.launch.py map_name:=my_map
```

**5. Base navigation**
```bash
ros2 launch base_navigation nav2_navigation.launch.py
```

**6. Base laser (SICK front+rear)** — needed by `base_scan_proximity`; skip
if you don't need the close-proximity safety net
```bash
ros2 launch omni_base_laser_sensors laser_sick-571.launch.py
```

**7. Base scan proximity** (needs the base laser up first)
```bash
ros2 launch base_scan_proximity base_scan_proximity.launch.py
```

**8. School traffic control**
```bash
ros2 launch school_traffic_control school_traffic_control.launch.py
```

**9. Benchmark logging** — only if this run is for the paper's data collection
```bash
ros2 launch benchmark_logging benchmark_logging.launch.py
```

Verify:
```bash
ros2 topic echo /fix                 # GPS
ros2 topic hz /amcl_pose             # localization alive
```

---

## Run jetson drone (10.68.0.208, user `tiago-jetson`)

Log in **interactively** (not `ssh host 'command'`) so `~/.bashrc` on this
machine actually gets sourced — it exports the `RMW_IMPLEMENTATION`/
`CYCLONEDDS_URI` needed to reach the robot over DDS:
```bash
ssh tiago-jetson@10.68.0.208
```
then in that shell for every command below:
```bash
export ROS_DOMAIN_ID=2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**1. IMU** (Xsens MTi — data storage only, not consumed by the state machine)
```bash
ros2 launch xsens_mti_imu_bringup xsens_mti_imu_bringup.launch.py
```

**2. robot_audio** (external speaker is physically connected to this
machine) — first point PulseAudio at the USB audio adapter, since the
onboard/built-in sink on this Jetson is a non-functional stub:
```bash
pactl set-default-sink "$(pactl list short sinks | grep -i usb | awk '{print $2}' | head -1)"
ros2 launch robot_audio robot_audio.launch.py
```

**3. Drone perception** (VisDrone/YOLO over RTMP) — its own venv, not a
`ros2 launch`:
```bash
source ~/visdrone_deployment/venv/bin/activate
cd ~/ros2_ws/src/outdoor_social_robot/perception/drone_traffic_perception
python main.py
```

Verify:
```bash
ros2 topic echo /drone_vehicle_detections
ros2 topic echo /drone_vehicle_detections_link_status
ros2 action send_goal /play_audio robot_audio_msgs/action/PlayAudio "{file_name: stop_audio.mp3}"
```

Note: GPS does **not** run on this machine — it moved to the robot (see
above). This Jetson's arm64 apt build of `ros-humble-ublox-gps` is broken
(missing `diagnostic_updater` shared lib); building from source works but
publishes fix data on `/ublox_gps/fix` instead of `/fix`.

---

## Run jetson perception (10.68.0.206, user `pal`)

```bash
ssh pal@10.68.0.206
```
then in that shell for every command below:
```bash
export ROS_DOMAIN_ID=2
```

Each node needs its **own** venv, so each of these two also needs its own
terminal regardless of wanting to check them individually — only one venv
can be active per shell.

**1. traffic_object_detection** (head_front_camera, camera+LiDAR fusion)
```bash
source /opt/ros/humble/setup.bash
source ~/venvs/yolo_ros/bin/activate
source ~/ros2_ws/install/setup.bash
ros2 launch traffic_object_detection detect_jetson.launch.py \
    publish_calibrated_tf:=false \
    config_file:="$HOME/ros2_ws/src/outdoor_social_robot/perception/traffic_object_detection/config/detection_head_front.yaml"
```

**2. vehicle_plate_detection_fastalpr** (head_front_camera is already its default)
```bash
source /opt/ros/humble/setup.bash
source ~/venvs/plate_detection_fastalpr/bin/activate
source ~/ros2_ws/install/setup.bash
ros2 launch vehicle_plate_detection_fastalpr detect_jetson.launch.py
```

Both default to `/perception/plate_detection_enabled` and
`/perception/traffic_object_detection_enabled` being OFF at rest —
`school_traffic_control` toggles them; that's expected until a vehicle
actually enters range.
