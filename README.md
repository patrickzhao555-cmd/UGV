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
- field map, confirmed marker detection, or manual goal received: switch immediately to target navigation
- target reached: treat the marker as reached inside `TARGET_ACCEPT_RADIUS_M` (default `0.9144`, about 1 yard), then loiter/scan instead of stopping in competition mode
- UAV landing flag received: reduce command speed while continuing obstacle avoidance

Simulate an ESP/UAV mission flag:

```bash
ros2 topic pub --once /ugv/mission_flag std_msgs/msg/String \
"{data: '{\"state\":\"landing\",\"source\":\"bench\"}'}"
```

Marker vision baseline:

1. Put marker photos in `ros2_ws/src/ugv_perception/training/marker_images/`.
2. Train the lightweight ORB model after building/sourcing the workspace:

```bash
cd ~/ugv_project/ros2_ws
ros2 run ugv_perception train_marker_model \
  --image-dir src/ugv_perception/training/marker_images \
  --model-out src/ugv_perception/models/marker_orb_model.npz
```

3. Run with marker vision enabled:

```bash
START_MARKER_VISION=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

The marker vision node publishes confirmed detections to `/ugv/marker_detection`.
Navigation already consumes that topic as the highest-priority target source and
publishes `/ugv/uav_flag` so the ESP/UAV side can stop scanning or prepare
landing. Tune `MARKER_MIN_GOOD_MATCHES`, `MARKER_CONFIRMATION_FRAMES`, and
`MARKER_CONFIRMATION_RADIUS_M` if bench data shows missed detections or false
positives.

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
- `/ugv/marker_detection`: camera/CV marker goal input consumed by navigation
- `/ugv/mission_flag`: manual/ESP/UAV mission state such as `landing`, `leaving`, or `scanning`
- `/ugv/uav_flag`: target-found flag for ESP/UAV handoff
- `/ugv_nav_cmd`: navigation command JSON sent to the motor bridge
- `/ugv_nav_status`: navigation, mission, pose, planner, and command status
- `/ugv/debug_status`: compact one-line debug status
- `/motor_controller/status`: motor bridge connection and command state

## Sensor and Pathing Flow

Current competition data flow:

1. `zed_sync_node` publishes `/zed/depth`, `/zed/imu`, and optionally `/zed/image`.
2. `lidar_sync_node` publishes timestamp-corrected `/scan/synced`.
3. `motor_controller_bridge` publishes `/encoder_ticks_stamped`.
4. `fusion_node` aligns LiDAR, ZED depth/IMU, and encoder ticks into `/sensors/nav_frame`.
5. `ugv_nav_dual_mode.py` uses `/sensors/nav_frame` plus `/ugv/field_map`, `/ugv/marker_detection`, `/ugv_goal`, and `/ugv/mission_flag`.
6. Marker vision, when enabled, publishes confirmed target detections to `/ugv/marker_detection`.
7. Navigation publishes `/ugv_nav_cmd`, `/ugv_nav_status`, and `/ugv/uav_flag` when the marker is found locally.

Target priority is: confirmed camera marker, then ESP/UAV field-map marker, then
manual `/ugv_goal`. Live LiDAR/ZED obstacles are always allowed to add new
obstacles during pathing because the field map may be incomplete.

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
MOTOR_PWM_SLEW_RATE_US_PER_S=2400.0
TARGET_ACCEPT_RADIUS_M=0.9144
MARKER_MIN_GOOD_MATCHES=18
MARKER_CONFIRMATION_FRAMES=2
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
