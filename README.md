# UGV Autonomous Ground Vehicle Stack

ROS 2 workspace for the UGV competition platform. The current stack brings up
LiDAR, ZED 2i depth, motor-controller serial I/O, sensor fusion, navigation,
bench-test tooling, and competition-mode startup behavior for a 15 x 15 yard
field.

The active development branch for Jetson/Nano testing is:

```bash
nav2-inspired-mini-controller
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
git checkout nav2-inspired-mini-controller
```

Pull the latest branch:

```bash
cd ~/ugv_project
git fetch origin
git checkout nav2-inspired-mini-controller
git pull --ff-only origin nav2-inspired-mini-controller
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

Launch with the visual dashboard enabled from a VNC desktop:

```bash
cd ~/ugv_project/ros2_ws
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 START_YOLO_OBSTACLES=true START_DEBUG_DASHBOARD=true \
MOTOR_PORT=/dev/ttyACM0 LIDAR_PORT=/dev/ttyUSB0 MOTOR_DRY_RUN=false \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

The dashboard opens one OpenCV window with ZED image, YOLO boxes, marker
candidate boxes, ZED depth map, local LiDAR/fused obstacle view, a session map,
and the current navigation reason/status. Press `s` in the window to save a
screenshot under `~/ugv_dashboard_screenshots`; press `q` or `Esc` to close it.
Set `DASHBOARD_CAMERA_SEARCH_DEPTH_M=0.30` to match the measured camera marker
search depth, and `DASHBOARD_UPDATE_HZ=4.0` if the Nano UI feels heavy.

If you need a desktop on a headless Nano, install and start TigerVNC:

```bash
sudo apt update
sudo apt install -y tigervnc-standalone-server tigervnc-common xfce4 xfce4-goodies dbus-x11
vncpasswd
mkdir -p ~/.vnc
cat > ~/.vnc/xstartup <<'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
startxfce4 &
EOF
chmod +x ~/.vnc/xstartup
vncserver -localhost no :1 -geometry 1920x1080 -depth 24
```

Then connect from your laptop to `<nano-ip>:5901`. If you launch from SSH
instead of an opened VNC terminal, export the VNC display first:

```bash
export DISPLAY=:1
```

Every new terminal that uses ROS commands should source the same overlays:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
```

## Bench Test Modes

Direct drivetrain direction test. Use this with the UGV lifted for the first
run, and do not run the full navigation bringup at the same time:

```bash
cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
source install/setup.bash

ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=forward raw:=0.22 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=true invert_right_command:=true
```

Swap `motion:=forward` for `backward`, `turn_left`, `turn_right`,
`left_forward`, `right_forward`, or `sequence` to isolate drivetrain problems.
The tester sends repeated `STOP` packets after the motion and prints encoder
delta so command inversion and encoder inversion can be checked quickly.

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

YOLO debug now also publishes the raw bounding boxes, accept/reject reason,
confidence, and depth estimate on `/sensors/yolo_semantic_debug`; the visual
dashboard uses that topic to show exactly what YOLO thinks it sees.

ZED depth obstacle filtering:

- Fusion now uses a ground-aware ZED depth filter before publishing
  `/sensors/zed_obstacle_points`.
- For each image row in the depth ROI it estimates the floor/background depth,
  rejects the floor itself, then keeps connected vertical clusters that are
  closer than that row model.
- This is designed for the low camera height: traffic cones, buckets, Amazon
  boxes, chair/table legs, and other upright obstacles become obstacle points,
  while the red/orange floor at the bottom of the depth map is ignored.
- Front depth clearance is computed from those filtered obstacle clusters inside
  the forward corridor instead of from raw floor pixels.

Active scan recovery:

- Indoor navigation does not give up on a forward route just because one depth
  frame contains orange/red obstacles. It still allows short forward probing.
- If front/depth blockage evidence repeats for several frames, or the path
  planner repeatedly fails while the front corridor is constrained, the UGV
  turns in place toward the clearer side to build a wider LiDAR/ZED view.
- This is meant for classroom-style clutter: if the current view is blocked but
  the right side is an open aisle, the robot should scan and retarget instead of
  sitting still or pushing blindly into the chair/table cluster.

Nav2-inspired mini local controller:

- This branch keeps the existing global search, map, target, and marker logic,
  but replaces the old blocky local motion selection with a continuous velocity
  controller enabled by default: `NAV_CONTINUOUS_CONTROL_ENABLED=true`.
- The controller samples forward `(v_mps, omega_radps)` trajectories over about
  1 second, simulates the 30 inch chassis through the local obstacle field, and
  scores progress, gap width, obstacle clearance, turning change, and speed
  smoothness.
- It builds a local polar gap/costmap view from LiDAR, ZED depth obstacle
  points, and optional YOLO semantic obstacle points. The old six-sector
  clearance values are still published for debugging, but they are no longer the
  normal path-choice mechanism in continuous mode.
- A collision-monitor layer remains on top: near obstacles cap speed or command
  `STOP`, while wider local path choice stays with the sampled controller.
- Reverse motion stays disabled for normal indoor runs. If a route is behind the
  robot, the controller should turn in place and then drive forward instead of
  backing up blind with the 180 degree front LiDAR view.
- Acceleration limits and low-pass smoothing happen at the velocity layer, so
  the UGV should roll forward and arc around clutter instead of doing
  `FORWARD/TURN/FORWARD/TURN` action blocks.

Useful depth-filter overrides:

```bash
FUSION_DEPTH_GROUND_FILTER_ENABLED=true
FUSION_DEPTH_PROJECTION_STRIDE_PX=8
FUSION_DEPTH_GROUND_MIN_DELTA_M=0.18
FUSION_DEPTH_GROUND_RATIO=0.88
FUSION_DEPTH_OBSTACLE_MIN_COMPONENT_HEIGHT_PX=14
FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M=0.50
NAV_ACTIVE_SCAN_ENABLED=true
NAV_ACTIVE_SCAN_CONFIRM_STEPS=4
NAV_ACTIVE_SCAN_STEPS=5
NAV_ACTIVE_SCAN_FRONT_CLEAR_M=1.35
```

Useful continuous-controller overrides:

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=true
NAV_CONTINUOUS_MAX_SPEED_MPS=0.36
NAV_CONTINUOUS_MAX_OMEGA_RPS=1.15
NAV_CONTINUOUS_HORIZON_S=1.10
NAV_CONTINUOUS_ACCEL_LIMIT_MPS2=0.35
NAV_CONTINUOUS_OMEGA_ACCEL_LIMIT_RPS2=1.80
NAV_CONTINUOUS_LOWPASS_ALPHA=0.55
NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M=1.35
NAV_CONTINUOUS_STOP_CLEARANCE_M=0.58
NAV_CONTINUOUS_LATENCY_BUFFER_S=0.25
```

If a ground test feels too aggressive, first try
`NAV_CONTINUOUS_MAX_SPEED_MPS=0.28` and
`NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M=1.50`. If you need to compare against the
older local planner, launch with `NAV_CONTINUOUS_CONTROL_ENABLED=false`.

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
- `/sensors/yolo_semantic_debug`: YOLO boxes, classes, confidence, depth, and accept/reject reason
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

Visual debugging:

- `START_DEBUG_DASHBOARD=true` starts `ugv_debug_dashboard`.
- The dashboard subscribes to `/zed/image`, `/zed/depth`, `/sensors/nav_frame`,
  `/sensors/yolo_semantic_debug`, `/ugv/marker_vision_debug`,
  `/sensors/synced_summary`, and `/ugv_nav_status`.
- It is meant for VNC/X11. Without `DISPLAY`, it logs a warning and does not
  call `cv2.imshow`.

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

Current map design:

- Navigation keeps an obstacle-focused `known_costmap` from live LiDAR and ZED/YOLO points.
- Indoor search also keeps coarse visited cells so it prefers not-yet-visited areas.
- It does not yet use a persistent "camera searched this exact FOV area" map for
  planning decisions. The dashboard now visualizes a session-local camera search
  wedge map using the current pose, 110 degree horizontal FOV, and the configured
  `DASHBOARD_CAMERA_SEARCH_DEPTH_M` so we can debug coverage before feeding it
  into the planner.

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
NAV_CONTINUOUS_CONTROL_ENABLED=true
NAV_CONTINUOUS_MAX_SPEED_MPS=0.36
NAV_CONTINUOUS_MAX_OMEGA_RPS=1.15
NAV_CONTINUOUS_HORIZON_S=1.10
NAV_CONTINUOUS_ACCEL_LIMIT_MPS2=0.35
NAV_CONTINUOUS_OMEGA_ACCEL_LIMIT_RPS2=1.80
NAV_CONTINUOUS_LOWPASS_ALPHA=0.55
NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M=1.35
NAV_CONTINUOUS_STOP_CLEARANCE_M=0.58
NAV_CONTINUOUS_LATENCY_BUFFER_S=0.25
NAV_FRONT_SAFETY_MARGIN_M=0.10
NAV_REAR_SAFETY_MARGIN_M=0.08
NAV_LOCAL_PLAN_INFLATION_M=0.08
NAV_ACTIVE_SCAN_ENABLED=true
NAV_ACTIVE_SCAN_CONFIRM_STEPS=4
NAV_ACTIVE_SCAN_STEPS=5
NAV_ACTIVE_SCAN_FRONT_CLEAR_M=1.35
FUSION_DEPTH_GROUND_FILTER_ENABLED=true
FUSION_DEPTH_GROUND_MIN_DELTA_M=0.18
FUSION_DEPTH_GROUND_RATIO=0.88
FUSION_DEPTH_OBSTACLE_MIN_COMPONENT_HEIGHT_PX=14
FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M=0.50
INVERT_LEFT_COMMAND=true
INVERT_RIGHT_COMMAND=true
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

If `/ugv_nav_status` shows `forward_command_has_opposite_encoder_signs` or
`turn_command_has_linear_odom_check_encoder_inversion` repeatedly, navigation
now stops with an encoder calibration fault instead of continuing to twitch.
That means the robot is seeing impossible odometry for the command it just sent;
fix motor/encoder inversion before tuning path planning.

The current 30 inch chassis defaults both command inversions to `true` in
`jetson_bringup.sh`, because the motor wiring maps positive PWM to physical
reverse on both sides. If a future wiring change makes `FORWARD` go backward
again, flip both command inversion flags together before tuning navigation:

```bash
INVERT_LEFT_COMMAND=true INVERT_RIGHT_COMMAND=true
```

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

Indoor search is tuned to prefer slow forward/arc-forward motion when the front
corridor is clear enough, even if a side direction looks more open. This helps
the robot pass chair/table gaps instead of repeatedly spinning in place.

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
