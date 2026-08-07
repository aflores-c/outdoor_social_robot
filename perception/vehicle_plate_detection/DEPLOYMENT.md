# Jetson Deployment

`vehicle_plate_detection` needs `torch` + `ultralytics` (plate-detector YOLO)
and `easyocr` (plate text OCR). These must **not** go into the `yolo_ros`
venv used by `traffic_object_detection`/`semantic_segmentation` — `easyocr`
pulls in `scikit-image` and friends, which force-upgrade `numpy` to a version
`tensorflow` in that venv can't tolerate (`tensorflow 2.19.0 requires
numpy<2.2.0`, `easyocr` pulls `numpy 2.2.6`). This happened once already and
had to be reverted. This package gets its own venv instead, so its dependency
resolution can never touch `yolo_ros` again.

Steps below follow Ultralytics' official Jetson install guide
(https://docs.ultralytics.com/guides/nvidia-jetson/, JetPack 6.1 section) for
the `torch`/`torchvision` wheels — this Jetson is on JetPack 6.1, confirmed
from `yolo_ros`'s already-installed `torch==2.10.0`/`torchvision==0.25.0`,
which match that section's wheels exactly.

**Automated:** steps 1-7 below are scripted in
`scripts/setup_jetson_venv.sh` — it's idempotent/resumable (safe to re-run
after the mid-setup reboot or any failure), so `./scripts/setup_jetson_venv.sh`
is the fastest path. The manual steps are spelled out below for reference /
troubleshooting. Either way, finish with steps 8-9 (build + launch).

## 1. Create the venv

```bash
python3 -m venv --system-site-packages ~/venvs/plate_detection
```

`--system-site-packages` is required so the venv can still see `rclpy`,
`cv_bridge`, `ament_index_python`, etc., which are tied to the system Python
via apt — same reasoning as `yolo_ros` (see
`traffic_object_detection/DEPLOYMENT.md`).

## 2. Install ultralytics (official Jetson sequence)

```bash
source ~/venvs/plate_detection/bin/activate

sudo apt update
sudo apt install python3-pip -y
pip install -U pip
pip install ultralytics[export]
```

Reboot before installing the Jetson-specific torch wheels below (per the
official guide) — pick a moment that won't interrupt anything else running
on this Jetson:

```bash
sudo reboot
```

## 3. Install the JetPack 6.1 torch/torchvision wheels

After reboot, re-activate the venv and install the Jetson-matched wheels
(this overwrites whatever generic `torch` `ultralytics[export]` pulled in
above — same versions already proven working in `yolo_ros`):

```bash
source ~/venvs/plate_detection/bin/activate

pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.10.0-cp310-cp310-linux_aarch64.whl
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.25.0-cp310-cp310-linux_aarch64.whl
```

## 4. Install cuDSS (torch 2.10.0 dependency on Jetson)

```bash
wget https://developer.download.nvidia.com/compute/cudss/0.7.1/local_installers/cudss-local-tegra-repo-ubuntu2204-0.7.1_0.7.1-1_arm64.deb
sudo dpkg -i cudss-local-tegra-repo-ubuntu2204-0.7.1_0.7.1-1_arm64.deb
sudo cp /var/cudss-local-tegra-repo-ubuntu2204-0.7.1/cudss-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update && sudo apt-get -y install cudss
```

## 5. Install easyocr

Safe to install freely now — this venv is isolated from `yolo_ros`, so
`easyocr` upgrading `numpy` here can't break `tensorflow` elsewhere:

```bash
pip install easyocr
```

## 6. Verify torch sees CUDA

```bash
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

Should print `CUDA available: True`. If not, stop here and recheck steps 3–4
before continuing — a CPU-only plate detector will be far too slow for
real-time use.

## 7. Install colcon inside the venv

```bash
pip install --ignore-installed --force-reinstall colcon-common-extensions
```

`--ignore-installed --force-reinstall` is required: with
`--system-site-packages`, pip sees the system's `colcon-core` as "already
satisfied" and skips installing anything locally, so no `bin/colcon` script
ever gets created in the venv. Forcing the reinstall creates a real
`~/venvs/plate_detection/bin/colcon`.

## 8. Build

Source order matters — the last-sourced environment wins on `PATH`, so
activate the venv *after* the ROS underlay:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/plate_detection/bin/activate
```

Invoke colcon as a `python3` module explicitly, to guarantee the build uses
the venv's Python (which determines the shebang baked into the installed
node script):

```bash
cd ~/ros2_ws
rm -rf build/vehicle_plate_detection install/vehicle_plate_detection
python3 -m colcon build --symlink-install --packages-select vehicle_plate_detection
```

Verify the shebang before launching:

```bash
head -1 install/vehicle_plate_detection/lib/vehicle_plate_detection/plate_detector_node
# must show:  #!/home/pal/venvs/plate_detection/bin/python3
```

If it still shows `/usr/bin/python3` or `.../yolo_ros/...`, the build didn't
run under this venv — recheck `which python3` / `echo $VIRTUAL_ENV` before
rebuilding.

## 9. Launch

Each new shell needs the full chain sourced again, in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/plate_detection/bin/activate
source ~/ros2_ws/install/setup.bash

ros2 launch vehicle_plate_detection detect_jetson.launch.py
```

Make sure the license-plate model weights are present before launching —
`~/ros2_ws/src/.../vehicle_plate_detection/models/license_plate_detector.pt`
must exist *before* step 8's `colcon build` runs, since `setup.py` only
installs what's physically there at build time.
