#!/usr/bin/env bash
#
# Automates DEPLOYMENT.md steps 1-7: creates a DEDICATED venv for
# vehicle_plate_detection (torch/ultralytics/easyocr) on the Jetson,
# separate from the yolo_ros venv used by traffic_object_detection /
# semantic_segmentation — easyocr's deps force-upgrade numpy past what
# tensorflow in yolo_ros tolerates, so this package must never share
# that venv.
#
# Idempotent / resumable: safe to re-run after the required mid-setup
# reboot, or after any failure — already-completed steps are skipped.
#
# Usage:
#   ./setup_jetson_venv.sh
#
# After this script finishes, follow DEPLOYMENT.md steps 8-9 to build
# the ROS package against this venv and launch it.
#
# See also: setup_jetson_venv_fastalpr.sh, a separate/parallel venv for
# the fast-alpr-based alternative node (plate_detector_fastalpr_node.py) —
# this script and that one are independent, so both pipelines can be
# built and tested side by side.

set -eo pipefail

VENV_DIR="${PLATE_DETECTION_VENV:-$HOME/venvs/plate_detection}"
MARKER_DIR="$VENV_DIR/.setup_markers"

step_done() { [ -f "$MARKER_DIR/$1" ]; }
mark_done() { touch "$MARKER_DIR/$1"; }

echo "=== vehicle_plate_detection Jetson venv setup ==="
echo "Target venv: $VENV_DIR"
echo

# 1. Create venv
if [ ! -d "$VENV_DIR" ]; then
    echo "[1/7] Creating venv at $VENV_DIR"
    python3 -m venv --system-site-packages "$VENV_DIR"
else
    echo "[1/7] venv already exists at $VENV_DIR — skipping creation"
fi

mkdir -p "$MARKER_DIR"

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# 2. apt prereqs + ultralytics[export]
#
# apt update/install on this Jetson (custom "auvidea-agx-orin" carrier
# board) always trips over unrelated, pre-existing broken nvidia-l4t-*
# packages ("does not match any known boards") and exits non-zero, even
# though the packages we actually need get handled fine — confirmed
# harmless (same thing happens for the yolo_ros venv, which works). Don't
# let `set -e` treat that as fatal here.
if ! step_done 02_ultralytics; then
    echo "[2/7] Installing apt prerequisites + ultralytics[export]"
    sudo apt update || echo "  (ignoring known-harmless apt exit code on this board)"
    sudo apt install -y python3-pip || echo "  (ignoring known-harmless apt exit code on this board)"
    pip install -U pip
    pip install "ultralytics[export]"
    mark_done 02_ultralytics
else
    echo "[2/7] ultralytics[export] already installed — skipping"
fi

# 3. Reboot checkpoint (required before installing the Jetson-specific
# torch/torchvision wheels, per Ultralytics' official Jetson guide)
if ! step_done 03_rebooted; then
    mark_done 03_rebooted
    echo
    echo "############################################################"
    echo "# Reboot required before installing the Jetson-specific"
    echo "# torch/torchvision wheels. Re-run this script after the"
    echo "# Jetson comes back up — it will resume from here."
    echo "############################################################"
    read -r -p "Press Enter to reboot now, or Ctrl+C to cancel and reboot manually later..." _
    sudo reboot
    exit 0
fi
echo "[3/7] Reboot checkpoint already passed — skipping"

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# 4. torch/torchvision Jetson wheels (JetPack 6.1 — confirmed to match
# this Jetson from yolo_ros's already-installed torch==2.10.0/torchvision==0.25.0)
if ! step_done 04_torch; then
    echo "[4/7] Installing JetPack 6.1 torch/torchvision wheels"
    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.10.0-cp310-cp310-linux_aarch64.whl
    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.25.0-cp310-cp310-linux_aarch64.whl
    mark_done 04_torch
else
    echo "[4/7] torch/torchvision wheels already installed — skipping"
fi

# 5. cuDSS (torch 2.10.0 dependency on Jetson)
if ! step_done 05_cudss; then
    echo "[5/7] Installing cuDSS"
    DEB_PATH="/tmp/cudss-local-tegra-repo-ubuntu2204-0.7.1_0.7.1-1_arm64.deb"
    wget -O "$DEB_PATH" \
        https://developer.download.nvidia.com/compute/cudss/0.7.1/local_installers/cudss-local-tegra-repo-ubuntu2204-0.7.1_0.7.1-1_arm64.deb
    sudo dpkg -i "$DEB_PATH"
    sudo cp /var/cudss-local-tegra-repo-ubuntu2204-0.7.1/cudss-*-keyring.gpg /usr/share/keyrings/
    # See the note on step 2 — ignore this board's known-harmless apt exit
    # code, but explicitly verify cudss itself actually landed.
    sudo apt-get update || echo "  (ignoring known-harmless apt exit code on this board)"
    sudo apt-get -y install cudss || echo "  (ignoring known-harmless apt exit code on this board)"
    dpkg -s cudss >/dev/null 2>&1 || { echo "ERROR: cudss did not actually install — check apt output above."; exit 1; }
    rm -f "$DEB_PATH"
    mark_done 05_cudss
else
    echo "[5/7] cuDSS already installed — skipping"
fi

# 6. easyocr — isolated from yolo_ros (no tensorflow here to conflict
# with), but easyocr's own unpinned `numpy` dependency still defaults to
# numpy>=2 if left unconstrained, which breaks the *system* cv_bridge and
# matplotlib (both precompiled against numpy 1.x's ABI, needed by
# plate_detector_node.py and ultralytics respectively). Constrain numpy
# alongside easyocr in the same pip call so the resolver picks
# numpy<2-compatible releases of easyocr's transitive deps
# (scikit-image/scipy/opencv-python-headless/etc.) from the start, rather
# than installing numpy>=2 versions of those and breaking cv_bridge/matplotlib.
if ! step_done 06_easyocr; then
    echo "[6/7] Installing easyocr (numpy<2 constrained)"
    pip install "numpy<2" easyocr
    mark_done 06_easyocr
else
    echo "[6/7] easyocr already installed — skipping"
fi

# 7. colcon inside the venv (needed so the eventual colcon build bakes
# this venv's Python into the node's shebang)
if ! step_done 07_colcon; then
    echo "[7/7] Installing colcon-common-extensions into the venv"
    pip install --ignore-installed --force-reinstall colcon-common-extensions
    mark_done 07_colcon
else
    echo "[7/7] colcon already installed in venv — skipping"
fi

echo
echo "=== Verifying torch sees CUDA ==="
if python3 -c "import torch; ok = torch.cuda.is_available(); print('CUDA available:', ok); raise SystemExit(0 if ok else 1)"; then
    echo "OK."
else
    echo "WARNING: torch does not report CUDA available — recheck steps 4/5 before continuing."
fi

echo
echo "=== Venv setup complete: $VENV_DIR ==="
echo "Next (DEPLOYMENT.md steps 8-9):"
echo "  1. Make sure the model weights exist at:"
echo "       <repo>/vehicle_plate_detection/models/license_plate_detector.pt"
echo "  2. Build, from your ROS 2 workspace root (source /opt/ros/humble"
echo "     BEFORE this venv):"
echo "       source /opt/ros/humble/setup.bash"
echo "       source $VENV_DIR/bin/activate"
echo "       rm -rf build/vehicle_plate_detection install/vehicle_plate_detection"
echo "       python3 -m colcon build --symlink-install --packages-select vehicle_plate_detection"
echo "  3. Verify the shebang:"
echo "       head -1 install/vehicle_plate_detection/lib/vehicle_plate_detection/plate_detector_node"
echo "       # must show: #!$VENV_DIR/bin/python3"
