#!/bin/bash
# Perception bringup on jetson 10.68.0.206 (user pal): traffic_object_detection
# (pedestrian + vehicle, camera+LiDAR fusion) and vehicle_plate_detection_fastalpr,
# both reading from head_front_camera.
#
# Each node needs its OWN venv (see traffic_object_detection/DEPLOYMENT.md and
# vehicle_plate_detection_fastalpr/DEPLOYMENT.md) — that's why they're launched
# as two separate subshells below rather than one sourced chain, since only one
# venv can be active at a time in a given shell.
#
# Adjust ROS2_WS below if this Jetson's workspace isn't at ~/ros2_ws.
#
# Logs land in ~/system_test_logs/*.log — tail -f any of them to check status.

set -u
export ROS_DOMAIN_ID=2
ROS2_WS=~/ros2_ws

LOGDIR=~/system_test_logs
mkdir -p "$LOGDIR"

echo "=== traffic_object_detection (head_front_camera, yolo_ros venv) ==="
(
  source /opt/ros/humble/setup.bash
  source ~/venvs/yolo_ros/bin/activate
  source "$ROS2_WS/install/setup.bash"
  export ROS_DOMAIN_ID=2
  exec ros2 launch traffic_object_detection detect_jetson.launch.py \
      publish_calibrated_tf:=false \
      config_file:="$ROS2_WS/src/outdoor_social_robot/perception/traffic_object_detection/config/detection_head_front.yaml"
) > "$LOGDIR/traffic_object_detection.log" 2>&1 < /dev/null &
disown
echo "started traffic_object_detection (pid $!) -> $LOGDIR/traffic_object_detection.log"

sleep 3

echo "=== vehicle_plate_detection_fastalpr (head_front_camera is already its default, plate_detection_fastalpr venv) ==="
(
  source /opt/ros/humble/setup.bash
  source ~/venvs/plate_detection_fastalpr/bin/activate
  source "$ROS2_WS/install/setup.bash"
  export ROS_DOMAIN_ID=2
  exec ros2 launch vehicle_plate_detection_fastalpr detect_jetson.launch.py
) > "$LOGDIR/vehicle_plate_detection_fastalpr.log" 2>&1 < /dev/null &
disown
echo "started vehicle_plate_detection_fastalpr (pid $!) -> $LOGDIR/vehicle_plate_detection_fastalpr.log"

echo
echo "Both nodes launched. Check logs in $LOGDIR/"
echo "Both default to /perception/plate_detection_enabled and"
echo "/perception/traffic_object_detection_enabled being OFF at rest —"
echo "school_traffic_control toggles them; this is expected until a vehicle"
echo "actually enters range."
