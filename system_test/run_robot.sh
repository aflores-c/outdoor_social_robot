#!/bin/bash
# Full system bringup on the robot's onboard PC (10.68.0.1, user pal).
# Run this directly on that machine (or via ssh pal@10.68.0.1 'bash -s' < this file).
#
# Assumes:
#   - A map has already been built and saved (see docs/system_test.md phase 1)
#     at ~/outdoor_robot_ws/src/outdoor_social_robot/localization/amcl_2d_localization/map/map.yaml
#   - pose_a_x/y/phi_deg and pose_b_x/y/phi_deg in school_traffic_control's
#     config yaml are already filled in (see docs/system_test.md phase 2.3) —
#     first run of a NEW map/site will not have these yet.
#   - PAL's own base laser bringup (omni_base_laser_sensors) is available
#     for base_scan_proximity's /scan input — comment out that block below
#     if you don't need the close-proximity safety net for this test.
#
# Logs land in ~/system_test_logs/*.log — tail -f any of them to check status.

# Deliberately no `set -u` — ROS 2's setup.bash itself references unset
# variables internally (e.g. AMENT_TRACE_SETUP_FILES), so nounset breaks
# sourcing it.
export ROS_DOMAIN_ID=2
source /opt/pal/alum/setup.bash
source ~/outdoor_robot_ws/install/setup.bash

LOGDIR=~/system_test_logs
mkdir -p "$LOGDIR"

launch_bg() {
  # $1 = log file name, rest = the ros2 launch command
  local name="$1"; shift
  nohup "$@" > "$LOGDIR/$name.log" 2>&1 < /dev/null &
  disown
  echo "started $name (pid $!) -> $LOGDIR/$name.log"
}

echo "=== GPS (SparkFun ZED-F9P + RTK/NTRIP corrections, SAPOS BW) ==="
echo "    ublox_gps/ntrip_client come from PAL's apt repo here (ros-humble-*),"
echo "    not built from source — fill in config/ntrip_credentials.yaml"
echo "    before launching if you haven't already."
launch_bg gps ros2 launch sparkfun_rtk_gps_bringup gps_rtk.launch.py

sleep 2

echo "=== Localization + navigation ==="
launch_bg velodyne        ros2 launch velodyne_vlp32c_bringup vlp32c_outdoor.launch.py
sleep 3
launch_bg scan_matcher    ros2 launch scan_matcher_bringup scan_matcher.launch.py
sleep 2
launch_bg amcl            ros2 launch amcl_2d_localization amcl_localization.launch.py
sleep 2
launch_bg base_navigation ros2 launch base_navigation nav2_navigation.launch.py

echo "=== Base 2D lidar (SICK front+rear) for base_scan_proximity ==="
echo "    Comment this block out if not testing the close-proximity safety net."
launch_bg omni_base_laser ros2 launch omni_base_laser_sensors laser_sick-571.launch.py
sleep 2
launch_bg base_scan_proximity ros2 launch base_scan_proximity base_scan_proximity.launch.py

echo "=== Traffic control stack ==="
echo "    robot_audio runs on the drone jetson (10.68.0.208), not here --"
echo "    that's where the external speaker is connected. See"
echo "    run_jetson_drone_208.sh. school_traffic_control's play_audio_action"
echo "    client reaches it fine over the network (same ROS_DOMAIN_ID)."
launch_bg school_traffic_control ros2 launch school_traffic_control school_traffic_control.launch.py

echo "=== Field-trial data collection ==="
echo "    Comment this out if this run isn't for the paper's data collection."
launch_bg benchmark_logging ros2 launch benchmark_logging benchmark_logging.launch.py

echo
echo "All nodes launched. Check logs in $LOGDIR/"
echo "Next: set the initial pose in RViz on the dev computer (10.68.0.209),"
echo "then drive the robot until localization converges (see docs/system_test.md)."
echo "Verify GPS with: ros2 topic echo /fix"
