# Jetson Deployment

`vehicle_plate_detection_fastalpr` needs `fast-alpr` (ONNX Runtime +
purpose-built plate detection/OCR models) instead of the original
`vehicle_plate_detection` package's `torch` + `ultralytics` + `easyocr`
stack. This is a deliberately **separate ROS package**, not a variant of
the original one, for two independent reasons:

1. **Isolation**: like `plate_detection`/`yolo_ros`, this needs its own
   dedicated venv so its dependency resolution can't touch either of
   those.
2. **colcon's per-package, one-venv-per-build model**: `colcon build`
   bakes a single active venv's Python into *every* console_script in a
   package. Two nodes needing different venvs can't live in the same
   package — a second build under a different venv would just overwrite
   the first node's shebang too. Hence: separate package, separate
   `colcon build --packages-select`, separate shebang.

Much lighter setup than the original package's — no Jetson-specific
`torch`/`torchvision` wheels, no cuDSS, no reboot required. Only
`onnxruntime-gpu` needs a Jetson-matched wheel; everything else is a
normal `pip install`.

**Automated:** steps 1-6 below are scripted in
`scripts/setup_jetson_venv_fastalpr.sh` — idempotent/resumable (safe to
re-run after any failure), so `./scripts/setup_jetson_venv_fastalpr.sh`
is the fastest path. The manual steps are spelled out below for
reference/troubleshooting. Either way, finish with steps 7-8 (build +
launch).

## 1. Create the venv

```bash
python3 -m venv --system-site-packages ~/venvs/plate_detection_fastalpr
```

`--system-site-packages` is required so the venv can still see `rclpy`,
`cv_bridge`, `ament_index_python`, etc., which are tied to the system
Python via apt — same reasoning as `yolo_ros`/`plate_detection` (see
`traffic_object_detection/DEPLOYMENT.md`).

## 2. apt prerequisites + pip

```bash
source ~/venvs/plate_detection_fastalpr/bin/activate

sudo apt update
sudo apt install python3-pip -y
pip install -U pip
```

## 3. Install the JetPack 6.1 onnxruntime-gpu wheel

Generic PyPI `onnxruntime-gpu` is x86_64-only — on Jetson it needs a
CUDA-matched ARM64 wheel. This is the JetPack 6.1 wheel already confirmed
for this Jetson (same source as the `torch`/`torchvision` wheels used
elsewhere in this repo):

```bash
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl
```

Install this **before** `fast-alpr` below, so `fast-alpr`'s own
`[onnx-gpu]` extra (which would try to pull a generic, non-Jetson wheel
from PyPI) is never invoked.

## 4. Install fast-alpr

Bare install — no extras. `fast-alpr` only requires *some* ONNX Runtime
backend to already be importable (`onnxruntime-gpu>=1.19.2`, satisfied
by the `1.23.0` wheel above); it doesn't pull one in itself unless you
ask for an extra:

```bash
pip install fast-alpr
```

## 5. Verify everything imports together

```bash
python3 - <<'PYEOF'
import traceback

def check(name, fn):
    try:
        fn()
        print(f"PASS: {name}")
    except Exception as e:
        print(f"FAIL: {name} -> {e!r}")
        traceback.print_exc()

def check_cuda():
    import onnxruntime as ort
    providers = ort.get_available_providers()
    if "CUDAExecutionProvider" not in providers:
        raise RuntimeError(f"no CUDAExecutionProvider, got: {providers}")

check("onnxruntime+cuda", check_cuda)
check("fast_alpr.ALPR", lambda: __import__("fast_alpr", fromlist=["ALPR"]).ALPR)
check("cv_bridge", lambda: __import__("cv_bridge"))
check("rclpy", lambda: __import__("rclpy"))
PYEOF
```

All four must print `PASS:` — `cv_bridge`/`rclpy` confirm
`--system-site-packages` is correctly exposing the ROS/system packages
this node also needs (same lesson learned the hard way on the original
`plate_detection` venv).

## 6. Install colcon inside the venv

```bash
pip install --ignore-installed --force-reinstall colcon-common-extensions
```

`--ignore-installed --force-reinstall` is required: with
`--system-site-packages`, pip sees the system's `colcon-core` as "already
satisfied" and skips installing anything locally, so no `bin/colcon`
script ever gets created in the venv. Forcing the reinstall creates a
real `~/venvs/plate_detection_fastalpr/bin/colcon`.

## 7. Build

Source order matters — the last-sourced environment wins on `PATH`, so
activate the venv *after* the ROS underlay:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/plate_detection_fastalpr/bin/activate
```

Invoke colcon as a `python3` module explicitly, to guarantee the build
uses the venv's Python (which determines the shebang baked into the
installed node script):

```bash
cd ~/ros2_ws
rm -rf build/vehicle_plate_detection_fastalpr install/vehicle_plate_detection_fastalpr
python3 -m colcon build --symlink-install --packages-select vehicle_plate_detection_fastalpr
```

Verify the shebang before launching:

```bash
head -1 install/vehicle_plate_detection_fastalpr/lib/vehicle_plate_detection_fastalpr/plate_detector_fastalpr_node
# must show:  #!/home/pal/venvs/plate_detection_fastalpr/bin/python3
```

If it still shows `/usr/bin/python3` or another venv's path, the build
didn't run under this venv — recheck `which python3` /
`echo $VIRTUAL_ENV` before rebuilding.

## 8. Launch

Each new shell needs the full chain sourced again, in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/plate_detection_fastalpr/bin/activate
source ~/ros2_ws/install/setup.bash

ros2 launch vehicle_plate_detection_fastalpr detect_jetson.launch.py
```

The `detector_model`/`ocr_model` params (default
`yolo-v9-t-384-license-plate-end2end` / `cct-xs-v2-global-model`) are
fast-alpr's pretrained ONNX models — they download and cache
automatically on first use, similar to EasyOCR's own model download
behavior in the original package, so the Jetson needs internet access
the first time this node actually starts.

## Running both packages side by side

`vehicle_plate_detection` (original) and `vehicle_plate_detection_fastalpr`
(this one) are fully independent — separate venvs, separate colcon
packages, separate node names. Both default to publishing
`plate_allowed_topic` as `/perception/plate_allowed` and subscribing to
the same `enabled_topic`, so:

- To compare **outputs on the same feed**, launch only one at a time
  (the second one's publish would just overwrite/race with the first on
  the shared topic) — swap between them with each package's own
  `detect_jetson.launch.py`.
- To run both **simultaneously** for a true side-by-side comparison,
  override `plate_allowed_topic` (and `enabled_topic` if you want
  independent activation, e.g. via `ros2 topic pub` to each) to distinct
  topics via each launch file's `config_file`/params, and watch both
  `.../last_plate` and `.../debug_image` topics in parallel. Note this
  means both heavy models run at once — expect the same GPU contention
  discussed in `school_traffic_control`'s perception-mode-switching
  design.
