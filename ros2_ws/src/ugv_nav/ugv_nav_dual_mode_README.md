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
  - Computes path and outputs commands like `FORWARD`, `TURN_LEFT`, `TURN_RIGHT`, `BACKWARD`, `STOP`

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
