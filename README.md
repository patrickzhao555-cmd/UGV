# UGV Autonomous Ground Vehicle Stack

ROS 2 workspace for the UGV competition platform: ZED 2i depth/image, 2D
LiDAR, encoder feedback, sensor fusion, marker vision, optional YOLO semantic
obstacles, a VNC-friendly debug dashboard, motor-controller serial I/O, and the
current Nav2-inspired mini local controller.

Active Jetson/Nano testing branch:

```bash
nav2-inspired-mini-controller
```

## Start Here

- English runbook: [ros2_ws/JETSON_BRINGUP_CHECKLIST.md](ros2_ws/JETSON_BRINGUP_CHECKLIST.md)
- 中文运行手册: [ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md](ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md)
- Architecture notes: [ros2_ws/STACK_ARCHITECTURE.md](ros2_ws/STACK_ARCHITECTURE.md)

The package-level READMEs are short module notes. The runbook above is the
single source of truth for pull, build, launch, VNC dashboard, motor tests,
YOLO, marker vision, tuning, and troubleshooting.

## Repository Layout

```text
ros2_ws/
  jetson_bringup.sh                     Main one-command launcher
  JETSON_BRINGUP_CHECKLIST.md           Full English runbook
  JETSON_BRINGUP_CHECKLIST_ZH.md        Full Chinese runbook
  STACK_ARCHITECTURE.md                 Layered stack design
  src/
    ugv_sensor_sync/                    ZED/LiDAR/encoder fusion and launch
    ugv_nav/                            Global search + continuous local control
    ugv_motor_controller/               Teensy serial bridge and direct motor test
    ugv_perception/                     Marker vision, YOLO assist, dashboard
    ugv_lidar/                          Legacy LiDAR helper package
    ugv_serial_odom/                    Legacy serial odometry package
    ugv_sim/                            Simulation package shell
    zed-ros2-wrapper/                   ZED ROS 2 wrapper sources
```

Generated ROS build folders (`ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`)
are intentionally ignored and should not be committed.

## Quick Pull, Build, Launch

On the Nano:

```bash
cd ~/ugv_project
git fetch origin
git checkout nav2-inspired-mini-controller
git pull --ff-only origin nav2-inspired-mini-controller

cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Indoor ground run with real motors, YOLO semantic obstacle assist, and the
debug dashboard:

```bash
cd ~/ugv_project/ros2_ws
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 \
START_YOLO_OBSTACLES=true START_DEBUG_DASHBOARD=true \
MOTOR_PORT=/dev/ttyACM0 LIDAR_PORT=/dev/ttyUSB0 MOTOR_DRY_RUN=false \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Use `MOTOR_DRY_RUN=true` for lifted or bench tests.

## What Runs

The current runtime flow is:

1. `zed_sync_node` publishes `/zed/image`, `/zed/depth`, `/zed/imu`, and `/zed/status`.
2. `lidar_sync_node` publishes timestamp-corrected `/scan/synced`.
3. `motor_controller_bridge` sends `/ugv_nav_cmd` to the Teensy and publishes `/encoder_ticks_stamped`.
4. `fusion_node` produces `/sensors/nav_frame`, `/sensors/synced_summary`, and obstacle point topics.
5. `marker_vision_node` optionally publishes confirmed marker targets on `/ugv/marker_detection`.
6. `yolo_semantic_obstacle_node` optionally publishes semantic obstacle inflation points.
7. `ugv_nav_dual_mode.py` keeps the global map/search/marker logic and uses a continuous DWA/RPP-style local controller.
8. `ugv_debug_dashboard` visualizes ZED image, depth, YOLO boxes, marker debug, LiDAR/fused points, session map, and nav decisions.

## Motion Control

The `nav2-inspired-mini-controller` branch keeps the existing global
map/search/marker behavior, but the local controller now samples forward
`(v_mps, omega_radps)` trajectories, simulates the 30 inch by 30 inch chassis,
scores progress, obstacle clearance, gap width, smoothness, and target
alignment, and publishes an explicit velocity-preferred command contract on
`/ugv_nav_cmd` while still including `raw_left`/`raw_right` for fallback.

The motor bridge remains backward compatible with raw commands. Closed-loop
wheel-speed PID is available but defaults off:

```bash
MOTOR_VELOCITY_CONTROL_ENABLED=false
NAV_EMIT_VELOCITY_COMMANDS=true
```

Enable `MOTOR_VELOCITY_CONTROL_ENABLED=true` only after lifted bench testing and
confirming fresh `/encoder_ticks_stamped` feedback. Inspect
`/motor_controller/status` for `control_mode`, target/measured wheel speeds,
PID terms, and stale-encoder safety stops.

Reverse motion is disabled for normal indoor runs. If a target is behind the
robot, the controller should turn in place and drive forward instead of backing
up blind with a front-only 180 degree LiDAR view.

## Safety Basics

- First real drivetrain test: lift the UGV.
- Do not run direct motor tests and full navigation at the same time; both write `/ugv_nav_cmd`.
- Current chassis defaults are `INVERT_LEFT_COMMAND=false` and `INVERT_RIGHT_COMMAND=true` in `jetson_bringup.sh`.
- If `FORWARD` physically drives backward, fix motor inversion before tuning navigation.
- If `FORWARD` makes one side drive backward and the other side drive forward, flip the command inversion on the wrong physical side first.
- If `/ugv_nav_status` repeatedly reports encoder sign warnings, navigation stops with an encoder calibration fault.
- Keep ROS Humble on `numpy==1.26.4`; NumPy 2.x can break `cv_bridge` and Ubuntu `matplotlib` binaries.

## Direct Motor Test

Use this to isolate motor direction and encoder signs:

```bash
cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
source install/setup.bash

ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=forward raw:=0.22 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=false invert_right_command:=true
```

Swap `motion:=forward` for `backward`, `turn_left`, `turn_right`,
`left_forward`, `right_forward`, or `sequence`.

## Main Topics

- `/sensors/nav_frame`: pathing-ready fused frame consumed by navigation
- `/sensors/synced_summary`: compact fusion/sensor health JSON
- `/sensors/yolo_semantic_obstacle_points`: optional semantic obstacle points
- `/sensors/yolo_semantic_debug`: YOLO boxes, classes, depth, and accept/reject reason
- `/ugv/marker_detection`: confirmed camera/CV marker target
- `/ugv_nav_cmd`: JSON motor command sent to the bridge
- `/ugv_nav_status`: planner, pose, command, sensor, and mission status
- `/ugv/debug_status`: one-line combined debug status
- `/motor_controller/status`: serial/motor/encoder bridge state

Use the runbook for full commands, parameter tables, TigerVNC setup, dashboard
usage, marker training, and troubleshooting.
