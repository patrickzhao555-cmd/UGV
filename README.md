# UGV Autonomous Ground Vehicle Stack

ROS 2 workspace for the UGV competition platform. The current stack brings up
LiDAR, ZED 2i depth, motor-controller serial I/O, sensor fusion, navigation,
bench-test tooling, and competition-mode startup behavior for a 15 x 15 yard
field.

The active development branch for Jetson/Nano testing is:

```bash
feature/motor-sync-nav-bringup
```

## Documentation

- Jetson bench and competition guide: [ros2_ws/JETSON_BRINGUP_CHECKLIST.md](ros2_ws/JETSON_BRINGUP_CHECKLIST.md)
- Chinese version: [ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md](ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md)
- Stack architecture notes: [ros2_ws/STACK_ARCHITECTURE.md](ros2_ws/STACK_ARCHITECTURE.md)
- Sensor sync package notes: [ros2_ws/src/ugv_sensor_sync/README](ros2_ws/src/ugv_sensor_sync/README)

## Repository Layout

```text
ros2_ws/
  jetson_bringup.sh                         Main Jetson launcher
  start_nav_test.sh                         Local nav test helper
  src/
    ugv_nav/                                Dual-mode navigation bridge and sim
    ugv_sensor_sync/                        ZED/LiDAR/encoder fusion and debug nodes
    ugv_motor_controller/                   Serial bridge to the motor controller
    ugv_perception/                         Perception helpers
    ugv_lidar/                              LiDAR package
    ugv_serial_odom/                        Serial odometry package
    zed-ros2-wrapper/                       ZED ROS 2 wrapper components
```

## Quick Start on Jetson/Nano

Pull the latest branch:

```bash
cd ~/ugv_project
git checkout feature/motor-sync-nav-bringup
git pull --ff-only origin feature/motor-sync-nav-bringup
```

Build:

```bash
cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Launch the full stack:

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## Bench Test Modes

Recommended stand/bench mode. This keeps motor output in dry-run and publishes
an automatic test goal:

```bash
cd ~/ugv_project/ros2_ws
BENCH_TEST=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Manual dry-run mode. Start the stack first, then publish a goal from another
terminal:

```bash
cd ~/ugv_project/ros2_ws
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash

ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 12.2, y: 12.0, z: 0.0}}"
```

Competition mock-map dry run:

```bash
cd ~/ugv_project/ros2_ws
COMPETITION_MODE=true START_MOCK_FIELD_MAP=true MOTOR_DRY_RUN=true \
START_CORNER=lower_left MOCK_MARKER_CELL=7,7 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Use `START_CORNER=upper_left`, `upper_right`, `lower_left`, or `lower_right`
to match the corner where the UGV is placed.

Competition behavior:

- no target yet: drive from the selected corner toward field center at reduced speed
- center reached, still no target: expand the search pattern outward from center while avoiding live LiDAR/ZED obstacles
- field map, marker detection, or manual goal received: switch immediately to target navigation
- UAV landing flag received: reduce command speed while continuing obstacle avoidance

Simulate an ESP/UAV mission flag:

```bash
ros2 topic pub --once /ugv/mission_flag std_msgs/msg/String \
"{data: '{\"state\":\"landing\",\"source\":\"bench\"}'}"
```

## Real Run Warning

Only run without `MOTOR_DRY_RUN=true` when the mechanical team has cleared the
vehicle for motion and the wheels are safe to command.

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## Main ROS Topics

- `/sensors/synced_summary`: fused LiDAR/ZED/encoder debug summary
- `/sensors/nav_frame`: synchronized nav input frame
- `/ugv_goal`: manual goal input in map coordinates
- `/ugv/field_map`: ESP/UAV or mock 15 x 15 field-map JSON
- `/ugv/marker_detection`: future camera/CV marker goal input
- `/ugv/mission_flag`: manual/ESP/UAV mission state such as `landing`, `leaving`, or `scanning`
- `/ugv_nav_cmd`: navigation command JSON sent to the motor bridge
- `/ugv_nav_status`: navigation, mission, pose, planner, and command status
- `/ugv/debug_status`: compact one-line debug status
- `/motor_controller/status`: motor bridge connection and command state

## Field Map Format

The competition pathing input is a 15 x 15 matrix for a 15 x 15 yard field,
where each cell is currently treated as 1 x 1 yard:

```json
{
  "type": "ugv_field_map_v1",
  "size": 15,
  "cell_size_yard": 1.0,
  "matrix_origin": "top_left",
  "world_origin": "lower_left",
  "matrix": [
    [2, 0, 0],
    [0, 1, 0],
    [0, 0, 3]
  ]
}
```

Legend:

- `0`: unknown or free
- `1`: obstacle
- `2`: UGV start/current cell
- `3`: marker destination

The field map is helpful but not trusted as complete. The navigation stack still
uses live LiDAR and ZED depth data to detect unmarked obstacles during motion.

## Safety and Debug Checks

Useful one-shot checks while the stack is running:

```bash
ros2 topic echo /sensors/synced_summary --once --full-length
ros2 topic echo /ugv/debug_status --once --full-length
ros2 topic echo /ugv_nav_status --once --full-length
ros2 topic echo /motor_controller/status --once --full-length
```

Useful tuning overrides:

```bash
MOTOR_PWM_SLEW_RATE_US_PER_S=1600.0
FUSION_IMU_SMOOTHING_ALPHA=0.25
USE_IMU_YAW=false
IMU_YAW_AXIS=z
IMU_YAW_SIGN=1.0
IMU_YAW_BLEND=0.25
```

IMU yaw fusion is available but disabled by default until the ZED IMU axis and
sign are confirmed on the physical robot. Encoder odometry remains the default
pose source; enabling `USE_IMU_YAW=true` blends gyro yaw rate into the tank-drive
heading estimate.

For detailed test procedures, expected outputs, parameter overrides, and bench
test recipes, use the full Jetson guide:
[ros2_ws/JETSON_BRINGUP_CHECKLIST.md](ros2_ws/JETSON_BRINGUP_CHECKLIST.md).
