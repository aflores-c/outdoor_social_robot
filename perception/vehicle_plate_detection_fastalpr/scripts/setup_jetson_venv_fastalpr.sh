#!/usr/bin/env bash
#
# Sets up a DEDICATED venv for the fast-alpr-based alternative node
# (plate_detector_fastalpr_node.py) — separate from BOTH the yolo_ros venv
# (traffic_object_detection/semantic_segmentation) AND the plate_detection
# venv (the original YOLO+EasyOCR plate_detector_node.py). All three stay
# fully independent so this package's two plate-detection pipelines can be
# built and tested side by side without touching each other.
#
# Much lighter than plate_detection's setup: fast-alpr runs on ONNX
# Runtime with purpose-built plate detection/OCR models, no torch, no
# ultralytics, no easyocr, no cuDSS, no reboot required.
#
# Idempotent / resumable: safe to re-run after any failure — already-
# completed steps are skipped.
#
# Usage:
#   ./setup_jetson_venv_fastalpr.sh
#
# After this script finishes, build plate_detector_fastalpr_node against
# this venv the same way DEPLOYMENT.md does for the original node, just
# with PLATE_DETECTION_FASTALPR_VENV / this venv's path instead.

set -eo pipefail

VENV_DIR="${PLATE_DETECTION_FASTALPR_VENV:-$HOME/venvs/plate_detection_fastalpr}"
MARKER_DIR="$VENV_DIR/.setup_markers"

step_done() { [ -f "$MARKER_DIR/$1" ]; }
mark_done() { touch "$MARKER_DIR/$1"; }

echo "=== vehicle_plate_detection (fast-alpr) Jetson venv setup ==="
echo "Target venv: $VENV_DIR"
echo

# 1. Create venv
if [ ! -d "$VENV_DIR" ]; then
    echo "[1/5] Creating venv at $VENV_DIR"
    python3 -m venv --system-site-packages "$VENV_DIR"
else
    echo "[1/5] venv already exists at $VENV_DIR — skipping creation"
fi

mkdir -p "$MARKER_DIR"

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# 2. apt prereqs + pip upgrade
#
# apt update/install on this Jetson (custom "auvidea-agx-orin" carrier
# board) always trips over unrelated, pre-existing broken nvidia-l4t-*
# packages ("does not match any known boards") and exits non-zero, even
# though the packages we actually need get handled fine — confirmed
# harmless (same thing happens for the other venvs in this repo, which
# work fine). Don't let `set -e` treat that as fatal here.
if ! step_done 02_pip; then
    echo "[2/5] Installing apt prerequisites + upgrading pip"
    sudo apt update || echo "  (ignoring known-harmless apt exit code on this board)"
    sudo apt install -y python3-pip || echo "  (ignoring known-harmless apt exit code on this board)"
    pip install -U pip
    mark_done 02_pip
else
    echo "[2/5] pip already up to date — skipping"
fi

# 3. onnxruntime-gpu (JetPack 6.1 wheel — same JetPack version already
# confirmed for this Jetson via the yolo_ros/plate_detection torch
# wheels elsewhere in this repo). Installed manually, ahead of fast-alpr,
# so fast-alpr's own [onnx-gpu] extra (which would pull a generic
# non-Jetson wheel from PyPI) is never used.
if ! step_done 03_onnxruntime; then
    echo "[3/5] Installing JetPack 6.1 onnxruntime-gpu wheel"
    pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl
    mark_done 03_onnxruntime
else
    echo "[3/5] onnxruntime-gpu already installed — skipping"
fi

# 4. fast-alpr itself — bare install (no [onnx-gpu] extra), since the
# Jetson-matched onnxruntime-gpu is already installed above and fast-alpr
# only requires *some* onnxruntime backend to already be importable
# (onnxruntime-gpu>=1.19.2, satisfied by 1.23.0).
if ! step_done 04_fastalpr; then
    echo "[4/5] Installing fast-alpr"
    pip install fast-alpr
    mark_done 04_fastalpr
else
    echo "[4/5] fast-alpr already installed — skipping"
fi

# 5. colcon inside the venv (needed so the eventual colcon build bakes
# this venv's Python into the node's shebang)
if ! step_done 05_colcon; then
    echo "[5/5] Installing colcon-common-extensions into the venv"
    pip install --ignore-installed --force-reinstall colcon-common-extensions
    mark_done 05_colcon
else
    echo "[5/5] colcon already installed in venv — skipping"
fi

echo
echo "=== Verifying onnxruntime sees CUDA ==="
if python3 -c "
import onnxruntime as ort
providers = ort.get_available_providers()
print('Available providers:', providers)
raise SystemExit(0 if 'CUDAExecutionProvider' in providers else 1)
"; then
    echo "OK."
else
    echo "WARNING: CUDAExecutionProvider not available — fast-alpr will fall back to CPU. Recheck step 3."
fi

echo
echo "=== Verifying fast_alpr imports ==="
python3 -c "from fast_alpr import ALPR; print('fast_alpr OK')"

echo
echo "=== Venv setup complete: $VENV_DIR ==="
echo "Next: build the ROS package against this venv:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $VENV_DIR/bin/activate"
echo "  rm -rf build/vehicle_plate_detection_fastalpr install/vehicle_plate_detection_fastalpr"
echo "  python3 -m colcon build --symlink-install --packages-select vehicle_plate_detection_fastalpr"
echo "  head -1 install/vehicle_plate_detection_fastalpr/lib/vehicle_plate_detection_fastalpr/plate_detector_fastalpr_node"
echo "  # must show: #!$VENV_DIR/bin/python3"
echo
echo "NOTE: this is a SEPARATE ROS package (vehicle_plate_detection_fastalpr)"
echo "from the original vehicle_plate_detection (YOLO+EasyOCR, plate_detection"
echo "venv) — deliberately so, since a single colcon build bakes ONE venv's"
echo "Python into every console_script in a package; two nodes needing"
echo "different venvs can't share a package."
