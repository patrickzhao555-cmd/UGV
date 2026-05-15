# UGV dual-mode navigation bridge

This script is a bridge between a pure simulation and a real UGV pipeline.

## What it does

- `--mode sim`
  - Opens a matplotlib GUI by default
  - Simulates virtual wheel encoders, 360 degree lidar, and forward ZED depth hits
  - Runs Hybrid A* on a growing known map
  - Prints high level control commands every step

- `--mode real`
  - No GUI
  - Reads the pathing-ready sensor frame from `ugv_sensor_sync`
  - Computes path and outputs velocity-layer commands that are converted to tank-drive raw motor values
  - Can fall back to legacy commands like `FORWARD`, `TURN_LEFT`, `TURN_RIGHT`, `BACKWARD`, `STOP`

## Expected real topics

If you use the built-in ROS 2 bridge, the script expects:

- `/sensors/nav_frame` as `ugv_sensor_sync/msg/NavSensorFrame`
  - includes lidar scan, ZED obstacle points, latest encoder ticks, and clearance summaries
- `/ugv_goal` as `geometry_msgs/PointStamped`
  - point in the map frame
- `/ugv/target` as `std_msgs/String`
  - ESP/UAV marker target coordinate in meters from the lower-left field origin
- `/ugv/field_map` as `std_msgs/String`
  - legacy optional 15 x 15 matrix for old bench tests

It publishes:

- `/ugv_nav_cmd` as `std_msgs/String`
  - JSON command such as
  - `{"mode":"TURN_LEFT","turn_deg":15.0,"move_m":0.0,...}`

## Useful commands

Run sim with GUI:

```bash
python ugv_nav_dual_mode.py --mode sim
```

Run sim without GUI:

```bash
python ugv_nav_dual_mode.py --mode sim --headless --max-steps 120
```

Run real mode through ROS 2:

```bash
python ugv_nav_dual_mode.py --mode real
```

Cap real-mode drive speed:

```bash
python ugv_nav_dual_mode.py --mode real --drive-speed-level 1
```

Speed levels are `1=25%`, `2=50%`, `3=75%`, and `4=100%`.
With `jetson_bringup.sh`, set `DRIVE_SPEED_LEVEL=1` before `bash jetson_bringup.sh`.

Run classroom/indoor marker search without a competition start corner:

```bash
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 bash jetson_bringup.sh
```

Indoor mode keeps ZED/marker vision on by default, starts from an internal room-center pose, ignores rear LiDAR returns, and chooses short forward/turn search goals instead of driving to the round3 field center. Defaults assume a 30 inch by 30 inch UGV, front-mounted LiDAR, `LIDAR_USED_FOV_DEG=180`, `LIDAR_OFFSET_X_M=0.30`, and `NAV_ALLOW_REVERSE=false`.

Continuous local control is enabled by default on the
`codex/nav2-inspired-mini-controller` branch:

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=true bash jetson_bringup.sh
```

Instead of choosing one discrete action block at a time, the controller samples
forward `(v_mps, omega_radps)` trajectories, simulates the robot over a short
horizon, scores progress, gap width, obstacle clearance, turning change, and
speed smoothness, then converts the selected velocity to left/right raw tank
commands. Acceleration limits and low-pass filtering live at the velocity layer,
so normal motion should be rolling arcs and smooth turns rather than repeated
stop-start blocks.

The controller uses a local polar gap/costmap view from LiDAR, ZED depth
obstacle points, and optional YOLO semantic points. A collision-monitor layer
still slows or stops the robot near obstacles, but ordinary path choice is made
by the sampled trajectories. Reverse motion is still disabled for indoor runs;
routes behind the robot are handled by turning in place and driving forward.

Useful overrides:

```bash
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

Use `NAV_CONTINUOUS_CONTROL_ENABLED=false` to compare against the old local
planner without changing the rest of the stack.

Generic marker detection is off by default because classroom objects can look marker-like. Re-enable it only for controlled marker tests:

```bash
MARKER_ENABLE_GENERIC_DETECTOR=true bash jetson_bringup.sh
```

The marker node also checks for a 12 inch square physical size, light border, and blocky grid-like interior before publishing generic detections. LiDAR remains the primary obstacle source; object detectors such as YOLO should stay optional because a missed table leg must not remove a collision obstacle from the safety layer.

YOLO semantic obstacle assist can be enabled separately when `ultralytics` is
installed. It publishes chair/table/person-style detections as extra obstacle
points for map inflation while LiDAR/ZED depth still own collision safety:

```bash
python3 -m pip install --user --force-reinstall "numpy==1.26.4"
python3 -m pip install --user "ultralytics" "numpy<2"
START_YOLO_OBSTACLES=true YOLO_MODEL_PATH=yolov8n.pt YOLO_DEVICE=auto bash jetson_bringup.sh
```

Keep NumPy on the 1.x line for ROS Humble compatibility with `cv_bridge` and
Ubuntu `matplotlib` packages.

Run real-mode logic from a JSON replay file:

```bash
python ugv_nav_dual_mode.py --mode real --replay-json sample_log.jsonl
```

Measure planner latency on the robot computer:

```bash
python3 src/ugv_nav/tools/benchmark_replan_latency.py --repeats 50
```

Live planner timing is also published in `/ugv_nav_status` as `plan_time_ms`.

## Why this matches your UGV better

- Internal planning map grows from sensor hits instead of starting with full map knowledge
- Differential / tank style control is used for command output
- Wheel encoder odometry drives the estimated pose
- Navigation now reads a stable middle-layer contract instead of individual raw sensor topics
- Goal input is separated from obstacle sensing, which matches the UAV gives goal and UGV handles navigation idea

## Odometry and Coverage Notes

The robot knows its heading from wheel encoder odometry by default. Optional ZED
IMU yaw blending exists but stays disabled until the IMU axis/sign are verified
on the physical chassis. If a forward command produces opposite left/right
encoder signs, or a pure turn produces strong linear odometry, the navigator now
refuses to integrate that bad pose update and stops after repeated warnings so
the chassis does not continue twitching on corrupted heading data.

The planner models the 30 inch by 30 inch footprint through `ROBOT_LENGTH_M`,
`ROBOT_WIDTH_M`, `ROBOT_TRACK_WIDTH_M`, obstacle inflation, and front/rear
safety margins. LiDAR is limited by `LIDAR_USED_FOV_DEG`; default indoor runs use
180 degrees and `NAV_ALLOW_REVERSE=false`, so planned reverse motion is disabled.

The live planning map is currently obstacle-focused: LiDAR and ZED/YOLO points
add obstacles, and indoor mode tracks coarse visited cells. Camera-FOV coverage
is visualized by `START_DEBUG_DASHBOARD=true`, but it is not yet a hard planner
objective.

Continuous-controller debug fields are published under `velocity_control` in
`/ugv_nav_status`. They include the chosen `v/omega`, safe sample count,
front-clearance speed cap, selected gap heading/depth, and collision-monitor
state.
