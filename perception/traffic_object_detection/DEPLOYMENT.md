# Jetson Deployment

Deploying `traffic_object_detection` on the Jetson requires `torch` +
`ultralytics`, which are installed via pip into a venv (they're not available
as apt packages for Jetson's CUDA build). This means `colcon build` must be
run in a way that bakes the venv's Python into the generated node's entry
point — this doc covers that, plus a network fix needed for stable
`/velodyne_points` reception.

## 1. Create the venv

```bash
python3 -m venv --system-site-packages ~/venvs/yolo_ros
```

`--system-site-packages` is required so the venv can still see `rclpy`,
`cv_bridge`, `ament_index_python`, etc., which are tied to the system Python
via apt.

## 2. Install deps + colcon inside the venv

```bash
source ~/venvs/yolo_ros/bin/activate
pip install ultralytics   # pulls torch — use the Jetson/NVIDIA-specific torch
                          # wheel first if you need CUDA support, plain pip
                          # torch has no CUDA for Jetson's aarch64
pip install --ignore-installed --force-reinstall colcon-common-extensions
```

`--ignore-installed --force-reinstall` is required: with
`--system-site-packages`, pip sees the system's `colcon-core` as "already
satisfied" and skips installing anything locally, so no `bin/colcon` script
ever gets created in the venv. Forcing the reinstall creates a real
`~/venvs/yolo_ros/bin/colcon`.

## 3. Build

Source order matters — the last-sourced environment wins on `PATH`, so
activate the venv *after* the ROS underlay:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/yolo_ros/bin/activate
```

Even with the venv active, `colcon` on `PATH` can still resolve to
`/usr/bin/colcon` in some setups. To be certain the build uses the venv's
Python (which determines the shebang baked into the installed node script),
invoke colcon as a module of `python3` explicitly:

```bash
cd ~/ros2_ws
rm -rf build/traffic_object_detection install/traffic_object_detection
python3 -m colcon build --symlink-install --packages-select traffic_object_detection
```

Verify the shebang is correct before launching:

```bash
head -1 install/traffic_object_detection/lib/traffic_object_detection/traffic_object_detector_node
# must show:  #!/home/pal/venvs/yolo_ros/bin/python3
```

If it still shows `/usr/bin/python3`, the build didn't run under the venv —
recheck `which python3` / `echo $VIRTUAL_ENV` before rebuilding.

If you've previously built this workspace *without* `--symlink-install`,
colcon will fail trying to convert an existing real install directory into a
symlink (`failed to create symbolic link ... Is a directory`). Fix by
cleaning that package's (or the whole workspace's) `build/`/`install/`/`log/`
before rebuilding.

## 4. Launch

Each new shell needs the full chain sourced again, in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/yolo_ros/bin/activate
source ~/ros2_ws/install/setup.bash
```

**Chest D455** (publishes its own calibrated TF):
```bash
ros2 launch traffic_object_detection detect_jetson.launch.py
```

**head_front_camera** (TF already exists via the robot's URDF +
`velodyne_vlp32c_bringup`, so skip republishing it):
```bash
ros2 launch traffic_object_detection detect_jetson.launch.py \
    publish_calibrated_tf:=false \
    config_file:=/home/pal/ros2_ws/src/outdoor_social_robot/perception/traffic_object_detection/config/detection_head_front.yaml
```

## 5. Known issue: `/velodyne_points` dropping to 0

Symptom: `img`/`info` counters in the node's debug log
(`[DEBUG] last 2s — img:X info:Y pc:Z synced_cb:W`) stay steady, but `pc`
periodically drops to 0 for extended stretches, recovering on its own or
after restarting the launch.

Root cause: the VLP-32C publishes multi-megabyte `PointCloud2` messages per
scan. The stock Linux UDP receive buffer
(`net.core.rmem_max`/`net.core.rmem_default`, default `212992` bytes / ~208KB
on the Jetson) is far smaller than one of these bursts. Whenever the receive
side can't drain the socket fast enough, the kernel silently drops the
overflow — with `sensor_data` (BEST_EFFORT) QoS there's no retransmit, so the
whole message is lost. This reproduces identically with a bare
`ros2 topic hz /velodyne_points`, independent of this node, confirming it's a
kernel/network issue, not application code.

Fix — raise the receive buffer size on the Jetson and persist it across
reboots:

```bash
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.rmem_default=8388608

echo -e "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608" | sudo tee /etc/sysctl.d/60-cyclonedds.conf
sudo sysctl --system
```

Verify after a reboot:
```bash
sysctl net.core.rmem_max net.core.rmem_default
```

## Reference: machine map (this deployment)

| IP | Role |
|---|---|
| 10.68.0.1   | robot (onboard main PC, runs the VLP-32C driver) |
| 10.68.0.55  | lidar (VLP-32C's own NIC, not a ROS host) |
| 10.68.0.206 | jetson robot — runs `traffic_object_detection` |
| 10.68.0.208 | jetson external (spare/different unit) |
| 10.68.0.209 | dev computer |

CycloneDDS discovery on this network uses explicit unicast peers
(`AllowMulticast=false` in `cyclonedds.xml`) rather than multicast, since
multicast discovery over WiFi links (e.g. the dev computer's connection) is
unreliable. If adding a new machine to this setup, make sure its
`cyclonedds.xml` `<Peers>` list includes the other hosts it needs to talk to,
and that the `<NetworkInterface>` name matches its actual active interface.
