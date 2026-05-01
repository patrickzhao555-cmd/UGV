# Jetson Bring-Up and Bench Test Guide

Chinese version: [JETSON_BRINGUP_CHECKLIST_ZH.md](JETSON_BRINGUP_CHECKLIST_ZH.md)

This guide assumes the repo is at `~/ugv_project` and this ROS 2 workspace is
`~/ugv_project/ros2_ws`.

Use `Ctrl+C` in the launch terminal to stop the stack.

## 1. Pull the Latest Code

```bash
cd ~/ugv_project
git checkout feature/motor-sync-nav-bringup
git pull --ff-only origin feature/motor-sync-nav-bringup
```

Confirm the workspace belongs to the repo:

```bash
cd ~/ugv_project/ros2_ws
git rev-parse --show-toplevel
```

Expected:

```text
/home/bluelule/ugv_project
```

## 2. Build

Run this after pulling code or changing Python/launch files.

```bash
cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Every new terminal should source the same setup files:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
```

## 3. One-Command Stack Launcher

The main entry point is:

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Useful device overrides:

```bash
LIDAR_PORT=/dev/ttyUSB0 MOTOR_PORT=/dev/ttyACM0 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

If motor direction or encoder direction is reversed:

```bash
INVERT_LEFT_COMMAND=true INVERT_LEFT_ENCODER=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## 4. Recommended Bench Mode

Use this when the UGV is on a stand. It starts the stack, enables dry-run motor
output, starts debug topics, and automatically publishes a test goal.

```bash
cd ~/ugv_project/ros2_ws
BENCH_TEST=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Change the automatic bench goal in meters:

```bash
BENCH_TEST=true BENCH_GOAL_X_M=6.86 BENCH_GOAL_Y_M=6.86 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

What good looks like:

- launch terminal shows `DRY RUN motor command`, not `Sent motor command`
- `/ugv/debug_status` shows `phase=manual` and `cmd=...`
- pose may stay fixed in dry-run because wheels are not actually commanded

## 5. Manual Dry-Run Mode

Use this when you want to launch the stack first, then publish a coordinate by
hand. Motors will not receive PWM.

Terminal 1:

```bash
cd ~/ugv_project/ros2_ws
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash

ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 12.2, y: 12.0, z: 0.0}}"
```

Coordinates are in meters. The current field model is 15 yards by 15 yards,
so the center is about `(6.86, 6.86)` meters.

## 6. Actual Motor Run on a Stand

Only use this when the robot is safely lifted. This sends real PWM to the motor
controller.

Terminal 1:

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash

ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 12.2, y: 12.0, z: 0.0}}"
```

Check the motor bridge:

```bash
ros2 topic echo /motor_controller/status --once --full-length
```

`TURN_LEFT` should produce left PWM below neutral and right PWM above neutral,
for example `1230, 1770`.

## 7. Ground Run

Use this only when the mechanical team confirms the platform is ready and the
area is clear.

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Then publish a goal from another terminal:

```bash
ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 6.86, y: 6.86, z: 0.0}}"
```

Keep a terminal open with:

```bash
ros2 topic echo /ugv/debug_status --full-length
```

## 8. Competition Mode

Competition mode can start from any corner. If no final target is known yet,
the UGV drives toward the field center. When it receives a map or marker
detection, it switches to the final target.

Corners:

- `lower_left`
- `lower_right`
- `upper_left`
- `upper_right`

Dry-run competition with a mock ESP/UAV map:

```bash
cd ~/ugv_project/ros2_ws
COMPETITION_MODE=true START_CORNER=lower_left START_MOCK_FIELD_MAP=true \
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Change mock marker cell:

```bash
COMPETITION_MODE=true START_CORNER=upper_right START_MOCK_FIELD_MAP=true \
MOCK_MARKER_CELL=7,7 MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Actual competition-style run without mock map:

```bash
COMPETITION_MODE=true START_CORNER=lower_left \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## 9. Send a Field Map Manually

The ESP/UAV map topic is `/ugv/field_map` as `std_msgs/String`.

Cell legend:

- `0`: unknown or free
- `1`: known obstacle
- `2`: UGV start
- `3`: marker destination

Matrix row `0` is the top/north edge. Column `0` is the left/west edge.

Example:

```bash
ros2 topic pub --once /ugv/field_map std_msgs/msg/String \
"{data: '{\"type\":\"ugv_field_map_v1\",\"source\":\"manual_test\",\"size\":15,\"cell_size_yard\":1.0,\"matrix\":[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,1,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,3,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,1,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[2,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]}'}"
```

## 10. Simulate Camera Marker Detection

If CV finds the marker before the UAV/ESP map is available, publish:

```bash
ros2 topic pub --once /ugv/marker_detection geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 6.86, y: 6.86, z: 0.0}}"
```

The UGV should publish a target-found flag:

```bash
ros2 topic echo /ugv/uav_flag --once --full-length
```

## 11. Debug Topics

Use these in separate sourced terminals:

```bash
ros2 topic echo /ugv/debug_status --full-length
```

```bash
ros2 topic echo /ugv_nav_status --full-length
```

```bash
ros2 topic echo /sensors/synced_summary --once --full-length
```

```bash
ros2 topic echo /zed/status --once --full-length
```

```bash
ros2 topic echo /encoder_ticks_stamped --once
```

```bash
ros2 topic echo /motor_controller/status --once --full-length
```

Important fields:

- `zed_valid` or `valid_depth_samples`: ZED depth is producing valid depth points
- `min_depth_range_m`: closest ZED depth point in the ROI
- `front_lidar_range_m`: closest LiDAR point in the front sector
- `front_clearance_m`: min of front LiDAR and ZED depth clearance
- `encoder_available`: fused sensor packet includes fresh encoder ticks
- `cmd`: current nav command
- `pose_m`: estimated odometry pose

## 12. Bench Test Procedures

### ZED Depth Occlusion Test

Start bench mode:

```bash
BENCH_TEST=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Put a box directly in front of the ZED camera and compare:

```bash
ros2 topic echo /sensors/synced_summary --once --full-length
```

Expected:

- `min_depth_range_m` decreases when the box is closer
- `front_clearance_m` follows `min_depth_range_m` if ZED sees the closest obstacle
- `valid_depth_samples` stays nonzero

### LiDAR Direction Test

Start bench mode, then place a box at known positions around the LiDAR:

```bash
ros2 topic echo /sensors/synced_summary --once --full-length
```

Test positions:

- front of the robot
- left side
- right side
- behind the robot

Expected:

- `front_lidar_range_m` should decrease only when the box is in the real front direction
- `min_lidar_range_m` may decrease for any direction because it is the 360-degree closest point

If a front box does not reduce `front_lidar_range_m`, the LiDAR zero-degree
direction may not match the robot front.

### Encoder Direction Test

Use actual motor output while the robot is lifted:

```bash
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Publish a goal, then watch encoder ticks:

```bash
ros2 topic echo /encoder_ticks_stamped
```

Expected:

- forward motion should make left and right averaged ticks change in the same direction
- turning should make left and right averaged ticks change in opposite directions

If signs are wrong, rerun with:

```bash
INVERT_LEFT_ENCODER=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

or:

```bash
INVERT_RIGHT_ENCODER=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

### Motor Command Mapping Test

Start motor bridge without nav so commands are manual:

```bash
START_NAV=false EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Publish a left turn:

```bash
ros2 topic pub -r 2 /ugv_nav_cmd std_msgs/msg/String \
"{data: '{\"mode\":\"TURN_LEFT\",\"raw_left\":-0.3,\"raw_right\":0.3}'}"
```

Expected:

- left PWM below `1500`
- right PWM above `1500`
- wheels turn opposite directions

Stop the publisher with `Ctrl+C`.

### Command Timeout Test

With `START_NAV=false`, publish one command:

```bash
ros2 topic pub --once /ugv_nav_cmd std_msgs/msg/String \
"{data: '{\"mode\":\"FORWARD\",\"raw_left\":0.3,\"raw_right\":0.3}'}"
```

Wait one second, then check:

```bash
ros2 topic echo /motor_controller/status --once --full-length
```

Expected:

- bridge sends or reports neutral PWM near `[1500, 1500]`
- command age is greater than the timeout

### Competition Mock Map Test

```bash
COMPETITION_MODE=true START_CORNER=lower_left START_MOCK_FIELD_MAP=true \
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Expected:

- no manual `/ugv_goal` is needed
- `/ugv_nav_status` shows a competition phase such as `startup_to_center`,
  `target_nav`, or `center_loiter`
- motor output says `DRY RUN`

## 13. Parameter Cheat Sheet

Environment variables for `jetson_bringup.sh`:

| Variable | Default | Purpose |
| --- | --- | --- |
| `EXTRA_SETUP_BASH` | empty | Extra workspace setup, usually `~/ugv_ws_albert/install/setup.bash` |
| `BENCH_TEST` | `false` | Enables dry-run, debug status, and automatic bench goal |
| `MOTOR_DRY_RUN` | `false` | Do not write PWM to the controller |
| `START_NAV` | `true` | Start/skip the navigation process |
| `START_MOTOR_CONTROLLER` | `true` | Start/skip motor bridge |
| `START_DEBUG_STATUS` | `true` | Start combined debug status node |
| `START_BENCH_GOAL` | `false` | Auto-publish `/ugv_goal` |
| `BENCH_GOAL_X_M` | `12.2` | Auto bench goal x in meters |
| `BENCH_GOAL_Y_M` | `12.0` | Auto bench goal y in meters |
| `COMPETITION_MODE` | `false` | Enable corner/startup-to-center mission logic |
| `START_CORNER` | `lower_left` | One of the four competition start corners |
| `START_MOCK_FIELD_MAP` | `false` | Publish mock 15x15 field map |
| `MOCK_MARKER_CELL` | `7,7` | Mock marker row,col |
| `LIDAR_PORT` | `/dev/ttyUSB0` | LiDAR serial device |
| `MOTOR_PORT` | `/dev/ttyACM0` | Teensy serial device |
| `FUSION_LIDAR_FRONT_FOV_DEG` | `70.0` | LiDAR front sector width |
| `INVERT_LEFT_COMMAND` | `false` | Flip left motor command |
| `INVERT_RIGHT_COMMAND` | `false` | Flip right motor command |
| `INVERT_LEFT_ENCODER` | `false` | Flip left encoder sign |
| `INVERT_RIGHT_ENCODER` | `false` | Flip right encoder sign |

## 14. Fast Failure Clues

- `/motor_controller/connected` stays `false`: wrong serial port, wrong baud, or Teensy not flashed
- `/encoder_ticks_stamped` is empty but connected is `true`: encoder wiring or firmware output issue
- `/sensors/synced_summary` has `encoder_available: false`: encoder timestamps are stale
- `/ugv_nav_cmd` is quiet after a goal: nav is not receiving `/sensors/nav_frame` or `/ugv_goal`
- `front_lidar_range_m` does not react to a front obstacle: LiDAR heading alignment likely needs correction
- `valid_depth_samples` is zero: ZED depth stream is missing or invalid
