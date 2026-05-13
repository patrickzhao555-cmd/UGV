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

Clone if the Nano does not have the repo yet:

```bash
cd ~
git clone https://github.com/patrickzhao555-cmd/UGV.git ugv_project
cd ~/ugv_project
git checkout feature/motor-sync-nav-bringup
```

Pull the latest branch:

```bash
cd ~/ugv_project
git checkout feature/motor-sync-nav-bringup
git pull --ff-only origin feature/motor-sync-nav-bringup
```

Source and rebuild after pulling:

```bash
cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If a launch still prints old-style logs such as `ULTRA is deprecated`, no
`debug_status_node`, or `allow_missing_goal=False` during `ROUND_MODE=indoor`,
clear the stale install overlay and rebuild once:

```bash
cd ~/ugv_project/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After a clean build, `ros2 interface show ugv_sensor_sync/msg/NavSensorFrame`
should show `sensor_msgs/Imu imu`. The runtime nodes tolerate older overlays
without that field, but seeing it confirms the current interface is active.

Launch indoor search with real motors:

```bash
cd ~/ugv_project/ros2_ws
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 \
MOTOR_PORT=/dev/ttyACM0 LIDAR_PORT=/dev/ttyUSB0 MOTOR_DRY_RUN=false \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Launch the same mode with YOLO semantic obstacle assist enabled:

```bash
cd ~/ugv_project/ros2_ws
python3 -m pip install --user --force-reinstall "numpy==1.26.4"
python3 -m pip install --user "ultralytics" "numpy<2"
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 START_YOLO_OBSTACLES=true \
MOTOR_PORT=/dev/ttyACM0 LIDAR_PORT=/dev/ttyUSB0 MOTOR_DRY_RUN=false \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

YOLO is optional and advisory. LiDAR and ZED depth still own collision safety;
YOLO only adds chair/table/person-style semantic obstacle points for more
conservative map inflation. Keep NumPy on the 1.x line on ROS Humble; a plain
`pip install ultralytics` can upgrade NumPy to 2.x, which breaks Ubuntu/ROS
`cv_bridge` and `matplotlib` binaries. If you see `_ARRAY_API not found` or
`numpy.core.multiarray failed to import`, rerun the NumPy 1.26.4 command above.

Every new terminal that uses ROS commands should source the same overlays:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
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

Competition mock-target dry run:

```bash
cd ~/ugv_project/ros2_ws
COMPETITION_MODE=true START_MOCK_FIELD_MAP=true MOTOR_DRY_RUN=true \
UGV_START_X_M=0.46 UGV_START_Y_M=0.46 MOCK_MARKER_CELL=7,7 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Set `UGV_START_X_M` and `UGV_START_Y_M` in meters from the field lower-left
corner (`x=0, y=0`). If those are omitted, `START_CORNER=upper_left`,
`upper_right`, `lower_left`, or `lower_right` is still used as a fallback.

Competition behavior:

- no target yet: drive from the measured start point or fallback corner toward field center
- center reached, still no target: expand the search pattern outward from center while avoiding live LiDAR/ZED obstacles
- ESP/UAV target coordinate, confirmed marker detection, legacy field map target, or manual goal received: switch immediately to target navigation
- target reached: treat the marker as reached inside `TARGET_ACCEPT_RADIUS_M` (default `0.9144`, about 1 yard), then loiter/scan instead of stopping in competition mode
- UAV landing flag received: reduce command speed while continuing obstacle avoidance; active motion commands are floored by `MIN_MOTION_RAW`

Round shortcuts for actual ground runs:

```bash
# Round 1: straight line, marker vision enabled by default, stop inside 1 yard of marker
ROUND_MODE=round1 EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh

# Round 2: straight line while UAV takes off/lands; CV or UAV marker switches to target nav
ROUND_MODE=round2 EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh

# Round 3: full 15 x 15 yard competition behavior from a measured start point
ROUND_MODE=round3 UGV_START_X_M=0.46 UGV_START_Y_M=0.46 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

For lifted or bench testing, add `MOTOR_DRY_RUN=true` to any of the round
commands.

Simulate an ESP/UAV mission flag:

```bash
ros2 topic pub --once /ugv/mission_flag std_msgs/msg/String \
"{data: '{\"state\":\"landing\",\"source\":\"bench\"}'}"
```

ESP/UAV target format:

The primary ESP/UAV target topic is `/ugv/target` as `std_msgs/String`. The
coordinate frame is the 15 yard x 15 yard field with the lower-left corner as
`x=0, y=0`; units are meters.

```bash
ros2 topic pub --once /ugv/target std_msgs/msg/String \
"{data: '{\"type\":\"ugv_target_v1\",\"source\":\"uav\",\"x\":6.86,\"y\":6.86,\"unit\":\"m\"}'}"
```

The nav bridge also accepts compact forms such as `{"x":6.86,"y":6.86}`,
`{"target":{"x_m":6.86,"y_m":6.86}}`, `[6.86,6.86]`, or `6.86,6.86`. It keeps
backward compatibility with the older `/ugv/field_map` matrix input, but the
UAV/ESP side no longer needs to send obstacle or UGV-current-position data.

Marker vision baseline:

1. Put marker photos in `ros2_ws/src/ugv_perception/training/marker_images/`.
2. Train the lightweight ORB model after building/sourcing the workspace. The
   trainer now tries to crop the black/white marker region first, so it avoids
   learning classroom floor or chair features when possible:

```bash
cd ~/ugv_project/ros2_ws
python3 src/ugv_perception/ugv_perception/marker_model_trainer.py \
  --image-dir src/ugv_perception/training/marker_images \
  --model-out src/ugv_perception/models/marker_orb_model.npz \
  --max-descriptors 65000
```

The direct Python command is preferred on the Jetson because `~/ugv_ws_albert`
may also contain an older `ugv_perception` package that shadows ROS executable
lookup.

3. Run with marker vision enabled:

```bash
START_MARKER_VISION=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Standalone live CV test, no motor/nav movement:

```bash
cd ~/ugv_project/ros2_ws
MARKER_VISION_TEST=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

The tester prints `SEARCHING ...` every few seconds when no marker is visible
and `MARKER DETECTED ... distance=... bearing=...` when the CV node sees the
marker. This is the fastest way to bench-check whether the current model and
generic dark/light detector are good enough before running pathing.

The bring-up launch starts marker vision from this workspace's source path, so
it is not affected by older `ugv_perception` executables in `~/ugv_ws_albert`.

The marker vision node publishes confirmed detections to `/ugv/marker_detection`.
Navigation already consumes that topic as the highest-priority target source and
publishes `/ugv/uav_flag` so the ESP/UAV side can stop scanning or prepare
landing. Runtime detection is hybrid: it first looks for a generic high-contrast
dark/light marker shape using region, edge, and color-neutrality cues, then
falls back to ORB model matching. This keeps it useful when the exact black/white
pattern changes between practice and competition. `/ugv/marker_detection` uses
map-frame `x/y` for navigation and carries camera depth distance in `point.z`;
the nav bridge includes that distance in `/ugv/uav_flag` and can treat the
marker as reached inside `TARGET_ACCEPT_RADIUS_M` even if map-frame odometry is
slightly noisy. Tune
`MARKER_GENERIC_MIN_AREA_FRAC`, `MARKER_GENERIC_MIN_CONTRAST`,
`MARKER_MIN_GOOD_MATCHES`, `MARKER_CONFIRMATION_FRAMES`, and
`MARKER_CONFIRMATION_RADIUS_M` if bench data shows missed detections or false positives.

YOLO semantic obstacle assist:

- disabled by default: `START_YOLO_OBSTACLES=false`
- enable with `START_YOLO_OBSTACLES=true`
- optional dependency: `python3 -m pip install --user "ultralytics" "numpy<2"`
- ROS Humble compatibility repair: `python3 -m pip install --user --force-reinstall "numpy==1.26.4"`
- default model: `YOLO_MODEL_PATH=yolov8n.pt`
- default device: `YOLO_DEVICE=auto`; set `YOLO_DEVICE=cpu` if the Nano CUDA/PyTorch stack is unstable
- default classes: `person,chair,couch,dining table,bench,potted plant,backpack,suitcase`
- output topic: `/sensors/yolo_semantic_obstacle_points`

Fusion merges YOLO semantic points into the existing navigation obstacle point
stream. A missed YOLO detection never removes a LiDAR/ZED obstacle; it only
means navigation falls back to the normal hard safety layer.

## Real Run Warning

Only run without `MOTOR_DRY_RUN=true` when the mechanical team has cleared the
vehicle for motion and the wheels are safe to command.

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Competition-style ground run with CV enabled:

```bash
cd ~/ugv_project/ros2_ws
ROUND_MODE=round3 UGV_START_X_M=0.46 UGV_START_Y_M=0.46 START_MARKER_VISION=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## Main ROS Topics

- `/sensors/synced_summary`: fused LiDAR/ZED/encoder debug summary
- `/sensors/nav_frame`: synchronized nav input frame
- `/sensors/yolo_semantic_obstacle_points`: optional YOLO semantic obstacle points
- `/ugv_goal`: manual goal input in map coordinates
- `/ugv/target`: ESP/UAV target marker coordinate in meters from the field lower-left corner
- `/ugv/field_map`: legacy optional 15 x 15 field-map JSON, still accepted for older tests
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
4. `fusion_node` aligns LiDAR, ZED depth/IMU, optional YOLO semantic obstacle points, and encoder ticks into `/sensors/nav_frame`.
5. `ugv_nav_dual_mode.py` uses `/sensors/nav_frame` plus `/ugv/target`, `/ugv/field_map`, `/ugv/marker_detection`, `/ugv_goal`, and `/ugv/mission_flag`.
6. Marker vision, when enabled, publishes confirmed target detections to `/ugv/marker_detection`.
7. Navigation publishes `/ugv_nav_cmd`, `/ugv_nav_status`, and `/ugv/uav_flag` when a target coordinate or local marker detection is accepted.

Target priority is: confirmed camera marker, then ESP/UAV target coordinate,
then legacy field-map marker, then manual `/ugv_goal`. Live LiDAR/ZED obstacles are always allowed to add new
obstacles during pathing because the target coordinate does not include obstacles. When a live
obstacle blocks the current route, it is inserted into the costmap; the planner
checks whether the near-future path conflicts, replans around the obstacle, and
then naturally follows the new shortest available route back toward the target.

## Target Format

The competition target input is a meter coordinate in a bottom-left-origin field
frame. The field is still 15 yards x 15 yards internally (`13.716 m x 13.716 m`).

```json
{
  "type": "ugv_target_v1",
  "source": "uav",
  "unit": "m",
  "frame": "field_bottom_left",
  "x": 6.86,
  "y": 6.86
}
```

The old matrix field map is still accepted for bench testing, but obstacles from
UAV/ESP are no longer required or expected. The navigation stack uses live LiDAR
and ZED depth data to detect obstacles during motion.

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
MIN_SPEED_MPS=0.178816
MIN_MOTION_RAW=0.22
NAV_FRONT_SAFETY_MARGIN_M=0.10
NAV_REAR_SAFETY_MARGIN_M=0.08
NAV_LOCAL_PLAN_INFLATION_M=0.08
INVERT_LEFT_COMMAND=false
INVERT_RIGHT_COMMAND=false
INVERT_LEFT_ENCODER=false
INVERT_RIGHT_ENCODER=false
TARGET_ACCEPT_RADIUS_M=0.9144
MARKER_MIN_GOOD_MATCHES=18
MARKER_MODEL_MAX_DESCRIPTORS=65000
MARKER_CONFIRMATION_FRAMES=2
FUSION_IMU_SMOOTHING_ALPHA=0.25
USE_IMU_YAW=false
IMU_YAW_AXIS=z
IMU_YAW_SIGN=1.0
IMU_YAW_BLEND=0.25
```

If one side drives backward when a positive command should move it forward,
invert that side's motor command at launch. After changing command inversion,
lift the robot and check `/encoder_ticks_stamped`; if a physically forward wheel
motion makes that side's ticks decrease, invert that encoder side too.

Example for a reversed left drivetrain:

```bash
INVERT_LEFT_COMMAND=true
# Add INVERT_LEFT_ENCODER=true only if forward left-wheel rotation reports negative ticks.
```

If LiDAR/control latency makes the robot feel too close to obstacles, increase
the local safety margins before the run. A conservative indoor/lifted test
profile is:

```bash
NAV_FRONT_SAFETY_MARGIN_M=0.25
NAV_REAR_SAFETY_MARGIN_M=0.14
NAV_LOCAL_PLAN_INFLATION_M=0.16
```

If ZED is temporarily failing to open but LiDAR and encoders are healthy, the
fusion node can still publish nav frames from LiDAR + encoder data. This is
enabled by default through `FUSION_ALLOW_LIDAR_ONLY=true`. For a chassis/pathing
dry run without ZED or marker vision:

```bash
ROUND_MODE=round3 UGV_START_X_M=0.46 UGV_START_Y_M=0.46 \
MOTOR_PORT=/dev/ttyACM0 LIDAR_PORT=/dev/ttyUSB0 \
START_ZED=false START_MARKER_VISION=false MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

IMU yaw fusion is available but disabled by default until the ZED IMU axis and
sign are confirmed on the physical robot. Encoder odometry remains the default
pose source; enabling `USE_IMU_YAW=true` blends gyro yaw rate into the tank-drive
heading estimate.

For detailed test procedures, expected outputs, parameter overrides, and bench
test recipes, use the full Jetson guide:
[ros2_ws/JETSON_BRINGUP_CHECKLIST.md](ros2_ws/JETSON_BRINGUP_CHECKLIST.md).
