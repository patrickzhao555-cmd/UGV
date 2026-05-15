# ugv_nav_dual_mode

Navigation bridge and simulator.

For full pull/build/launch instructions, use the consolidated runbooks:

- English: `ros2_ws/JETSON_BRINGUP_CHECKLIST.md`
- Chinese: `ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md`

## Responsibilities

`ugv_nav_dual_mode.py` keeps the global search, map, target, marker, and mission
state logic. On the current branch it uses a Nav2-inspired continuous local
controller by default.

The real-mode node consumes:

- `/sensors/nav_frame`
- `/ugv_goal`
- `/ugv/target`
- `/ugv/field_map` for legacy bench tests
- `/ugv/marker_detection`
- `/ugv/mission_flag`

It publishes:

- `/ugv_nav_cmd`
- `/ugv_nav_status`
- `/ugv/uav_flag`

## Continuous Local Controller

Default:

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=true
```

The controller samples forward `(v_mps, omega_radps)`, simulates short
trajectories through the local obstacle field, scores target progress,
clearance, polar gap width, heading, and smoothness, then applies acceleration
limits and low-pass filtering before converting velocity into tank-drive raw
commands.

The local safety check uses the actual rolled-out arc for each sampled command,
so staggered obstacles can be handled as a sequence of short S-curve decisions.
For the 30 inch chassis, the default tight-gap parameters are:

```bash
ROBOT_OBSTACLE_BUFFER_M=0.025
NAV_LOCAL_PLAN_INFLATION_M=0.0
NAV_CONTINUOUS_GAP_BUFFER_M=0.025
FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M=0.42
NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M=1.05
NAV_CONTINUOUS_STOP_CLEARANCE_M=0.48
```

Reverse is disabled for normal indoor runs:

```bash
NAV_ALLOW_REVERSE=false
LIDAR_USED_FOV_DEG=180.0
```

Use this to compare with the older action-block local planner:

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=false
```

## Simulator

Headless smoke test:

```bash
python3 ros2_ws/src/ugv_nav/ugv_nav_dual_mode.py --mode sim --headless --max-steps 120
```

Legacy local-planner comparison:

```bash
python3 ros2_ws/src/ugv_nav/ugv_nav_dual_mode.py \
  --mode sim --headless --max-steps 80 --continuous-control-enabled false
```

## Notes

- Heading comes from encoder odometry by default.
- Optional ZED IMU yaw-rate blending exists but stays disabled until axis/sign are verified.
- The chassis footprint is modeled with robot length, width, track width, obstacle inflation, and safety margins.
- The live map is obstacle-focused. Dashboard camera-coverage visualization is available, but camera coverage is not yet a hard planner objective.
