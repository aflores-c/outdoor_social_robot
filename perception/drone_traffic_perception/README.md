# drone_traffic_perception

## Input source

`main.py` supports two input paths, selected with one flag near the top
of `if __name__ == "__main__":` — same as the original script:

```python
USE_RTMP = False   # local video file (LOCAL_VIDEO) — current default
USE_RTMP = True    # RTMP stream (RTMP_URL)
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_traffic_perception
source ~/ros2_ws/install/setup.bash
```

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/visdrone_deployment/venv/bin/activate
cd ~/ros2_ws/src/drone_traffic_perception
python main.py
```

## Verify the topic (in another terminal)

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /drone_vehicle_detections
```
