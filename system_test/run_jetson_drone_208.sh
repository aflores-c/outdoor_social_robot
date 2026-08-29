#!/bin/bash
# Bringup on jetson 10.68.0.208 (user tiago-jetson): drone perception
# (VisDrone/YOLO over the RTMP feed) plus GPS and IMU, which are for
# data-storage only right now — not consumed by the state machine.
#
# Adjust ROS2_WS below if this Jetson's workspace isn't at ~/ros2_ws.
#
# Logs land in ~/system_test_logs/*.log — tail -f any of them to check status.

# Deliberately no `set -u` — ROS 2's setup.bash itself references unset
# variables internally (e.g. AMENT_TRACE_SETUP_FILES), so nounset breaks
# sourcing it.
export ROS_DOMAIN_ID=2
ROS2_WS=~/ros2_ws

LOGDIR=~/system_test_logs
mkdir -p "$LOGDIR"

source /opt/ros/humble/setup.bash
source "$ROS2_WS/install/setup.bash"

launch_bg() {
  local name="$1"; shift
  nohup "$@" > "$LOGDIR/$name.log" 2>&1 < /dev/null &
  disown
  echo "started $name (pid $!) -> $LOGDIR/$name.log"
}

echo "=== GPS (SparkFun ZED-F9P, no RTK corrections — data storage only) ==="
launch_bg gps ros2 launch sparkfun_rtk_gps_bringup gps_only.launch.py

sleep 2

echo "=== IMU (Xsens MTi — data storage only) ==="
launch_bg imu ros2 launch xsens_mti_imu_bringup xsens_mti_imu_bringup.launch.py

echo
echo "=== Drone perception (VisDrone/YOLO over RTMP) ==="
echo "    Runs in its own venv (~/visdrone_deployment/venv), not launched via"
echo "    'ros2 launch' — it's a plain script, see"
echo "    perception/drone_traffic_perception/README.md and main.py's"
echo "    USE_RTMP/RTMP_URL constants near the bottom of the file."
(
  source /opt/ros/humble/setup.bash
  source "$ROS2_WS/install/setup.bash"
  source ~/visdrone_deployment/venv/bin/activate
  cd "$ROS2_WS/src/outdoor_social_robot/perception/drone_traffic_perception"
  export ROS_DOMAIN_ID=2
  exec python main.py
) > "$LOGDIR/drone_traffic_perception.log" 2>&1 < /dev/null &
disown
echo "started drone_traffic_perception (pid $!) -> $LOGDIR/drone_traffic_perception.log"

echo
echo "All nodes launched. Check logs in $LOGDIR/"
echo "Verify with: ros2 topic echo /drone_vehicle_detections"
echo "             ros2 topic echo /drone_vehicle_detections_link_status"
