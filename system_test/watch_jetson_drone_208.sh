#!/bin/bash
# Opens a LOCAL tmux session (on this dev computer) with one window per
# program normally launched by run_jetson_drone_208.sh -- IMU, robot_audio,
# drone perception -- each SSH'd into jetson 10.68.0.208 and running in the
# FOREGROUND, not nohup'd/disowned. GPS no longer runs on this machine, see
# run_jetson_drone_208.sh -- it moved to the robot (10.68.0.1).
#
# This is a live-view/debug tool, not the persistent deployment path:
#   - You see each program's real output directly in its own window.
#   - Ctrl+C in a window stops just that one program.
#   - Detaching (Ctrl-b d) or closing this local tmux session, or losing
#     the network to .208, kills every one of these SSH connections and
#     therefore every remote program with it -- unlike run_jetson_drone_208.sh's
#     nohup+disown (or a *remote* tmux session), nothing here survives that.
#     Use run_jetson_drone_208.sh instead for a run that needs to survive
#     you disconnecting.
#
# Switch windows: Ctrl-b <number> or Ctrl-b w (window picker)
# Detach (kills everything, see above): Ctrl-b d
# Reattach if you didn't detach but got kicked out: tmux attach -t jetson208_watch

SESSION=jetson208_watch
HOST=tiago-jetson@10.68.0.208
# Remote workspace path is hardcoded (not a local ~-expanded variable) in
# every command string below -- ~/ros2_ws here would expand against THIS
# (local) machine's home directory at script-definition time, not the
# remote tiago-jetson user's, which silently broke every window earlier.

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists -- attaching."
  exec tmux attach -t "$SESSION"
fi

# -t forces a pty; bash -lc makes it a login shell so ~/.bashrc on .208
# (which exports the correct RMW_IMPLEMENTATION/CYCLONEDDS_URI) actually
# gets sourced -- a plain `ssh host 'command'` does NOT source .bashrc,
# which silently breaks cross-machine DDS discovery (hit this earlier).

tmux new-session -d -s "$SESSION" -n imu \
  "ssh -t -o StrictHostKeyChecking=accept-new $HOST 'bash -lc \"source /opt/ros/humble/setup.bash && source /home/tiago-jetson/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=2 && ros2 launch xsens_mti_imu_bringup xsens_mti_imu_bringup.launch.py\"'"

# No USB-sink detection needed here -- the persistent PulseAudio drop-in
# on .208 (/etc/pulse/default.pa.d/99-usb-audio-default.pa, see
# system_test.md) already sets it correctly on every PulseAudio start.
tmux new-window -t "$SESSION" -n robot_audio \
  "ssh -t -o StrictHostKeyChecking=accept-new $HOST 'bash -lc \"source /opt/ros/humble/setup.bash && source /home/tiago-jetson/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=2 && ros2 launch robot_audio robot_audio.launch.py\"'"

tmux new-window -t "$SESSION" -n drone_perception \
  "ssh -t -o StrictHostKeyChecking=accept-new $HOST 'bash -lc \"source /opt/ros/humble/setup.bash && source /home/tiago-jetson/ros2_ws/install/setup.bash && source ~/visdrone_deployment/venv/bin/activate && export ROS_DOMAIN_ID=2 && cd /home/tiago-jetson/ros2_ws/src/outdoor_social_robot/perception/drone_traffic_perception && python main.py\"'"

tmux select-window -t "$SESSION:imu"
exec tmux attach -t "$SESSION"
