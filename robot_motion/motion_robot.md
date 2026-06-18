# Robot Motion — Testing Guide

Full motion stack for the outdoor robot: Nav2-based base navigation and
play_motion2-based arm control, unified through the `robot_move` C++ library.

---

## Architecture overview

```
┌──────────────────────────────────────────────────────────────────┐
│  Your code / test node                                           │
│                                                                  │
│   RobotMove::move_base(x, y, phi)   RobotMove::move_arm(name)   │
└───────────────┬──────────────────────────────┬───────────────────┘
                │ action: /go_to_xy_phi         │ action: /play_motion2
                ▼                               ▼
  ┌─────────────────────────┐     ┌─────────────────────────────┐
  │  navigate_to_pose_server│     │     play_motion2_node        │
  │  (base_navigation pkg)  │     │     (play_motion2 pkg)       │
  └───────────┬─────────────┘     └──────────┬──────────────────┘
              │ action: /navigate_to_pose     │ ros2_control trajectories
              ▼                               ▼
  ┌─────────────────────────┐     ┌──────────────────────────────┐
  │  Nav2 stack             │     │  Joint trajectory controllers │
  │  planner + controller   │     │  (arm joints)                 │
  │  bt_navigator + behav.  │     └──────────────────────────────┘
  └─────────────────────────┘
```

### Packages

| Package | Role |
|---|---|
| `base_navigation` | Nav2 bringup + `navigate_to_pose_server` (wraps Nav2 into `GoToXYPhi` action) |
| `play_motion2` | Pre-recorded arm motion player, needs a motions YAML at launch |
| `robot_move` | C++ library — synchronous `move_base()` + `move_arm()` blocking calls |

---

## 1. Build

```bash
cd ~/outdoor_robot_ws
colcon build --packages-select base_navigation play_motion2 play_motion2_msgs robot_move
source install/setup.bash
```

---

## 2. Launch the navigation stack

Starts: planner server, controller server, bt_navigator, behavior server,
lifecycle manager, and the `navigate_to_pose_server` action bridge.

```bash
ros2 launch base_navigation nav2_navigation.launch.py
```

Confirm it is ready — all nodes should print `[lifecycle_manager] Lifecycle transition successful`:

```bash
ros2 node list | grep -E "planner|controller|bt_navigator|behavior|navigate_to_pose"
```

---

## 3. Launch play_motion2 (arm)

play_motion2 requires a YAML file describing the named motions and a motion
planner config. Replace `<path_to_motions.yaml>` with your robot's motions file.

```bash
ros2 launch play_motion2 play_motion2.launch.py \
    motions_file:=<path_to_motions.yaml>
```

With a custom motion planner config (optional):

```bash
ros2 launch play_motion2 play_motion2.launch.py \
    motions_file:=<path_to_motions.yaml> \
    motion_planner_config:=<path_to_motion_planner_config.yaml>
```

### Motions YAML format

```yaml
/play_motion2_mgr:
  ros__parameters:
    motions:
      home:
        joints: [arm_1_joint, arm_2_joint, arm_3_joint]
        positions: [0.0, 0.0, 0.0]   # one row = one waypoint
        times_from_start: [2.0]
        meta:
          name: Home
          usage: testing
          description: 'Move arm to home position'

      wave:
        joints: [arm_1_joint, arm_2_joint]
        positions: [0.5, 0.0,
                    0.0, 0.5,
                    0.5, 0.0]
        times_from_start: [1.0, 2.0, 3.0]
        meta:
          name: Wave
          usage: demo
          description: 'Wave motion'
```

---

## 4. Test navigation from the CLI

### Send a goal to `go_to_xy_phi` (degrees for phi)

```bash
ros2 action send_goal /go_to_xy_phi base_navigation/action/GoToXYPhi \
    "{x: 1.0, y: 0.0, phi: 0.0}"
```

| Field | Type | Description |
|---|---|---|
| `x` | float64 | Target X in the `map` frame (metres) |
| `y` | float64 | Target Y in the `map` frame (metres) |
| `phi` | float64 | Target heading (degrees, 0 = forward/+X) |

Watch feedback (distance remaining) and result:

```bash
ros2 action send_goal /go_to_xy_phi base_navigation/action/GoToXYPhi \
    "{x: 2.0, y: 1.0, phi: 90.0}" --feedback
```

Cancel an in-progress goal:

```bash
ros2 action send_goal /go_to_xy_phi base_navigation/action/GoToXYPhi \
    "{x: 100.0, y: 0.0, phi: 0.0}" &
sleep 2
ros2 action cancel /go_to_xy_phi
```

---

## 5. Test arm motions from the CLI

### List available motions

```bash
ros2 play_motion list
ros2 play_motion list --is-ready      # also shows readiness
```

Or via service:

```bash
ros2 service call /play_motion2/list_motions play_motion2_msgs/srv/ListMotions
```

### Get info about a motion

```bash
ros2 play_motion info home
ros2 play_motion info home --verbose   # shows joint positions and times
```

### Run a motion

```bash
ros2 play_motion run home
ros2 play_motion run wave --skip-planning   # skip MoveIt approach planning
```

Or directly via action:

```bash
ros2 action send_goal /play_motion2 play_motion2_msgs/action/PlayMotion2 \
    "{motion_name: 'home', skip_planning: false}"
```

### Check if a motion is ready

```bash
ros2 service call /play_motion2/is_motion_ready \
    play_motion2_msgs/srv/IsMotionReady "motion_key: 'home'"
```

---

## 6. Test base + arm together via `robot_move`

`RobotMove` is a C++ library. Link against it in your `CMakeLists.txt`:

```cmake
find_package(robot_move REQUIRED)
target_link_libraries(my_node robot_move::robot_move)
```

### API

```cpp
#include "robot_move/robot_move.hpp"

RobotMove robot("my_test_node");

// Move base to (x=2.0 m, y=1.0 m, heading=90°) — blocks until done
bool ok = robot.move_base(2.0, 1.0, 90.0);

// Execute named arm motion — blocks until done
bool ok = robot.move_arm("home");
```

### Sequential base + arm test

```cpp
RobotMove robot("test_sequence");

// Drive to position
if (robot.move_base(2.0, 0.0, 0.0)) {
    RCLCPP_INFO(..., "Arrived at target");
    // Then move arm
    robot.move_arm("wave");
}
```

### Parallel base + arm (from separate threads)

```cpp
RobotMove robot("test_parallel");

auto nav_thread = std::thread([&]() {
    robot.move_base(3.0, 0.0, 0.0);
});
auto arm_thread = std::thread([&]() {
    robot.move_arm("home");
});
nav_thread.join();
arm_thread.join();
```

> **Note:** `move_base` and `move_arm` are both blocking. Run them in
> separate threads to execute concurrently.

---

## 7. Full test sequence (terminal by terminal)

**Terminal 1 — Navigation stack:**
```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 launch base_navigation nav2_navigation.launch.py
```

**Terminal 2 — Arm motions:**
```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 launch play_motion2 play_motion2.launch.py \
    motions_file:=<path_to_motions.yaml>
```

**Terminal 3 — Test navigation:**
```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 action send_goal /go_to_xy_phi base_navigation/action/GoToXYPhi \
    "{x: 1.0, y: 0.0, phi: 0.0}" --feedback
```

**Terminal 4 — Test arm:**
```bash
source ~/outdoor_robot_ws/install/setup.bash
ros2 play_motion run home
```

---

## 8. Inspect active topics and actions

```bash
# Navigation
ros2 topic echo /navigate_to_pose/_action/status
ros2 action info /go_to_xy_phi
ros2 action info /navigate_to_pose

# Arm
ros2 action info /play_motion2
ros2 topic echo /joint_states

# TF — confirm map → odom → base_link chain is live
ros2 run tf2_ros tf2_echo map base_link
```

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `go_to_xy_phi` server not available | Navigation not launched | Run Terminal 1 |
| `NavigateToPose server not available` | lifecycle_manager not active | Check Nav2 logs for lifecycle errors |
| `play_motion2` server not available | Arm launch not running | Run Terminal 2 |
| Goal rejected by nav2 | No map / costmap not ready | Ensure localization and map server are running |
| Motion rejected (`disable_motion_planning=true, skip_planning=false`) | Config mismatch | Either set `skip_planning: true` or enable planning in `motion_planner_config.yaml` |
| `tf2` lookup error in nav2 | TF chain broken | Verify `map → odom → base_link` with `ros2 run tf2_ros tf2_echo map base_link` |
