#!/bin/bash
# Bringup on jetson 10.68.0.208 (user tiago-jetson): drone perception
# (VisDrone/YOLO over the RTMP feed), IMU (data-storage only right now —
# not consumed by the state machine), and robot_audio — this machine is
# where the external speaker is physically connected, not the robot
# itself. school_traffic_control (running on the robot) reaches
# robot_audio's play_audio action server fine over the network, same
# ROS_DOMAIN_ID.
#
# GPS no longer runs here — it moved to the robot (10.68.0.1), see
# run_robot.sh. This machine's ublox_gps was built from source
# (github.com/KumarRobotics/ublox.git, v3.0.0) and worked, but used
# private (~/fix) topics instead of the /fix every consumer expects; when
# we tried switching to the apt package here to match the robot/dev setup,
# the arm64 build of ros-humble-ublox-gps turned out to have a broken
# dependency (linked against a diagnostic_updater ABI with no .so in the
# currently archived version), so apt isn't viable here either. GPS was
# moved to the robot instead of fighting that.
#
# Adjust ROS2_WS below if this Jetson's workspace isn't at ~/ros2_ws.
#
# Logs land in ~/system_test_logs/*.log — tail -f any of them to check status.

# Deliberately no `set -u` — ROS 2's setup.bash itself references unset
# variables internally (e.g. AMENT_TRACE_SETUP_FILES), so nounset breaks
# sourcing it.
#
# RMW_IMPLEMENTATION/CYCLONEDDS_URI are deliberately NOT set here — this
# machine's ~/.bashrc already exports the correct ones (with the right DDS
# peers to reach the robot). Run this script from an interactive shell
# (plain `./run_jetson_drone_208.sh`, not via a non-login `bash -s <` pipe)
# so that environment is actually inherited; overriding it here previously
# pointed CYCLONEDDS_URI at the wrong file and broke discovery.
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

echo "=== IMU (Xsens MTi — data storage only) ==="
launch_bg imu ros2 launch xsens_mti_imu_bringup xsens_mti_imu_bringup.launch.py

echo
echo "=== robot_audio (external speaker on this machine) ==="
echo "    Defaulting PulseAudio output to the USB audio adapter -- the"
echo "    built-in/onboard sink on this Jetson is a non-functional stub"
echo "    not listed in 'aplay -l' (see this session's audio troubleshooting)."
USB_SINK=$(pactl list short sinks 2>/dev/null | grep -i usb | awk '{print $2}' | head -1)
if [ -n "$USB_SINK" ]; then
  pactl set-default-sink "$USB_SINK"
  pactl set-sink-volume "$USB_SINK" 150%
  echo "    default sink -> $USB_SINK"
  echo "    (this machine also has /etc/pulse/default.pa.d/99-usb-audio-default.pa"
  echo "    for persistence across PulseAudio restarts -- this is just a"
  echo "    belt-and-suspenders re-assert, not the primary fix; that file is"
  echo "    system config, not tracked in this repo, so it won't survive a"
  echo "    Jetson reflash -- recreate it if this machine is ever re-imaged.)"
else
  echo "    WARNING: no USB audio sink found via 'pactl list short sinks'"
  echo "    -- check the speaker is connected, robot_audio will likely be silent."
fi
launch_bg robot_audio ros2 launch robot_audio robot_audio.launch.py

sleep 2

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
echo "             ros2 action send_goal /play_audio robot_audio_msgs/action/PlayAudio \"{file_name: stop_audio.mp3}\""
