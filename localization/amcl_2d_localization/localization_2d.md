# 2D AMCL Localization — Testing Guide

2D localization stack for the outdoor robot: slam_toolbox-based mapping and
Nav2 AMCL-based localization using a saved occupancy map.

---

## Architecture overview

```
┌────────────────────────────────────────────────────────────────────────┐
│  Sensors                                                               │
│                                                                        │
│   /scan_outdoor (LaserScan)          TF: odom → base_footprint        │
└───────────┬──────────────────────────────────────────────┬─────────────┘
            │                                              │
            ▼  [MAPPING MODE]                             ▼  [LOCALIZATION MODE]
┌───────────────────────────┐              ┌──────────────────────────────────┐
│  async_slam_toolbox_node  │              │  map_server                      │
│  (slam_toolbox pkg)       │              │  (nav2_map_server pkg)           │
│                           │              │  Loads .yaml + .pgm from map/    │
│  Builds occupancy map     │              └─────────────┬────────────────────┘
│  TF: map → odom           │                            │ /map (OccupancyGrid)
│                           │              ┌─────────────▼────────────────────┐
│  Save with:               │              │  amcl                            │
│  map_saver_cli -f <name>  │              │  (nav2_amcl pkg)                 │
└───────────────────────────┘              │  Particle filter localization     │
                                           │  TF: map → odom                  │
                                           └─────────────┬────────────────────┘
                                                         │
                                           ┌─────────────▼────────────────────┐
                                           │  lifecycle_manager               │
                                           │  (autostart: manages both nodes) │
                                           └──────────────────────────────────┘
```

### Packages involved

| Package | Role |
|---|---|
| `amcl_2d_localization` | Bringup: launch files + config + map folder |
| `slam_toolbox` | Online async mapping, publishes `map → odom` TF |
| `nav2_map_server` | Serves a saved `.pgm` + `.yaml` occupancy map |
| `nav2_amcl` | Particle filter localizer against the loaded map |
| `nav2_lifecycle_manager` | Autostart lifecycle management for map_server + amcl |

---

## 1. Build

```bash
cd ~/outdoor_robot_ws
colcon build --packages-select amcl_2d_localization
source install/setup.bash
```

---

## 2. Mapping — build a new map

### 2.1 Launch the mapper

```bash
ros2 launch amcl_2d_localization mapping.launch.py
```

Key topics published by slam_toolbox:

| Topic | Type | Description |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Live occupancy map |
| `/slam_toolbox/scan_visualization` | `sensor_msgs/LaserScan` | Scan used by the mapper |

TF published: `map → odom`

### 2.2 Drive the robot

Drive the robot around the area to be mapped.  
Watch `/map` in RViz2 — cells fill in as the robot explores.

### 2.3 Save the map

When coverage looks complete, save from another terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

This creates `~/my_map.pgm` and `~/my_map.yaml`.

Copy them into the package's `map/` folder and rebuild:

```bash
cp ~/my_map.pgm ~/my_map.yaml \
    ~/outdoor_robot_ws/src/localization/amcl_2d_localization/map/

cd ~/outdoor_robot_ws
colcon build --packages-select amcl_2d_localization
```

### 2.4 Resume a previous mapping session

slam_toolbox can serialize its graph and resume:

```bash
# Save the current session (call this service while mapper is running)
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$HOME/outdoor_robot_ws/src/localization/amcl_2d_localization/map/my_map'}"

# Resume in a later session
ros2 launch amcl_2d_localization mapping.launch.py \
    load_map:=true  map_name:=my_map
```

---

## 3. Localization — use a saved map

### 3.1 Launch AMCL with the default map

The default map name is `map` → loads `map/map.yaml` + `map/map.pgm`.

```bash
ros2 launch amcl_2d_localization amcl_localization.launch.py
```

### 3.2 Select a different map at launch

```bash
ros2 launch amcl_2d_localization amcl_localization.launch.py \
    map_name:=my_map
```

### 3.3 Provide an initial pose at launch

```bash
ros2 launch amcl_2d_localization amcl_localization.launch.py \
    map_name:=my_map \
    initial_x:=2.5  initial_y:=1.0  initial_yaw:=1.57
```

Or set it at runtime via the `/initialpose` topic (e.g. with the "2D Pose Estimate" tool in RViz2):

```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
    '{header: {frame_id: map},
      pose: {pose: {position: {x: 2.5, y: 1.0},
                    orientation: {z: 0.707, w: 0.707}}}}'
```

### 3.4 Launch arguments summary

| Argument | Default | Description |
|---|---|---|
| `map_name` | `map` | Map file name (without `.yaml`) inside `map/` |
| `use_sim_time` | `false` | Use simulation clock |
| `initial_x` | `0.0` | Initial pose X in map frame (metres) |
| `initial_y` | `0.0` | Initial pose Y in map frame (metres) |
| `initial_yaw` | `0.0` | Initial pose yaw (radians) |

---

## 4. Required inputs

Both launch files need the same inputs:

| Topic / TF | Type | Source |
|---|---|---|
| `/scan_outdoor` | `sensor_msgs/LaserScan` | LiDAR driver |
| TF `odom → base_footprint` | — | Odometry (wheel encoders / `mobile_base_controller`) |

> AMCL does **not** need `/odom` topic — it only needs the TF.

---

## 5. Published outputs

### Mapping (`mapping.launch.py`)

| Topic / TF | Type | Description |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Live map being built |
| TF `map → odom` | — | Robot pose estimate |

### Localization (`amcl_localization.launch.py`)

| Topic / TF | Type | Description |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Served static map |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Best-estimate pose + covariance |
| `/particle_cloud` | `nav2_msgs/ParticleCloud` | All AMCL particles (for RViz2) |
| TF `map → odom` | — | Localization correction |

---

## 6. Verify localization is working

```bash
# Confirm map → odom → base_footprint TF chain is live
ros2 run tf2_ros tf2_echo map base_footprint

# Watch AMCL pose estimates
ros2 topic echo /amcl_pose

# Check particle count and spread (should converge after driving a bit)
ros2 topic hz /particle_cloud
```

In RViz2 add:

- **Map** → `/map`
- **PoseWithCovariance** → `/amcl_pose`
- **ParticleCloud** → `/particle_cloud`
- **LaserScan** → `/scan_outdoor` (should align with map walls when converged)

---

## 7. Full test sequence (terminal by terminal)

**Terminal 1 — Localization:**
```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 launch amcl_2d_localization amcl_localization.launch.py map_name:=my_map
```

**Terminal 2 — Set initial pose (if not launching with initial_x/y/yaw):**
```bash
source ~/outdoor_robot_ws/install/setup.bash
# Use RViz2 "2D Pose Estimate" button, or:
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
    '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}}}}'
```

**Terminal 3 — Monitor convergence:**
```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

**Terminal 4 — Navigation (after localization converges):**
```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 launch base_navigation nav2_navigation.launch.py
```

---

## 8. Map folder layout

```
map/
├── map.yaml          # Placeholder — replace with your saved map metadata
├── map.pgm           # Occupancy image (white=free, black=occupied, grey=unknown)
├── my_map.yaml       # Additional maps — selectable with map_name:=my_map
└── my_map.pgm
```

Map YAML format:
```yaml
image: my_map.pgm       # occupancy image filename
resolution: 0.05        # metres per pixel
origin: [0.0, 0.0, 0.0] # [x, y, yaw] of bottom-left pixel in map frame
negate: 0               # 0 = white is free, black is occupied
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `map_server` fails to load map | Map file not found in `map/` | Copy `.pgm` + `.yaml` to `map/` and rebuild |
| AMCL particles don't converge | Bad initial pose | Use RViz2 "2D Pose Estimate" tool to set a rough pose |
| `tf2` lookup fails (`map` → `odom`) | lifecycle_manager not activated | Check that `lifecycle_manager_localization` started (look for `Lifecycle transition successful` in logs) |
| Laser scan doesn't align with map | Wrong `scan_topic` or bad initial pose | Verify `/scan_outdoor` is publishing; set a better initial pose |
| `/map` not published | `map_server` not in active state | lifecycle_manager autostart should handle this; if not, call `change_state` |
| `slam_toolbox` crashes on start | Missing TF `odom → base_footprint` | Ensure odometry is running before launching the mapper |
| Map looks distorted or loopy | Loop-closure not triggering | Increase `loop_search_maximum_distance` in `config/slam_toolbox.yaml` |
