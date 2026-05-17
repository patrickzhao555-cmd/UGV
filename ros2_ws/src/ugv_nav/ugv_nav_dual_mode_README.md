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
limits and low-pass filtering.

Commands now make velocity intent explicit while retaining raw fallback fields:

```json
{
  "mode": "FORWARD",
  "command_type": "velocity",
  "v_mps": 0.18,
  "omega_radps": 0.35,
  "raw_left": 0.0,
  "raw_right": 0.0,
  "controller": "velocity"
}
```

Use `--emit-velocity-commands false` or `NAV_EMIT_VELOCITY_COMMANDS=false` to
make continuous-controller output raw-preferred again while preserving
`v_mps`/`omega_radps` for debug and simulation.

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

`NAV_LOCAL_COSTMAP_ENABLED=true` enables the rolling local costmap used by local
collision checks. It marks LiDAR/ZED obstacles, ray-clears both hit and no-hit
LiDAR beams from `ranges_m`/`angles_rad`, decays dynamic obstacles, inflates
local obstacles, and keeps field-map static obstacles separate from dynamic
clearing. Dynamic LiDAR/ZED observations are no longer written into the
persistent global planning map.

Useful local costmap overrides:

```bash
NAV_LOCAL_COSTMAP_WIDTH_M=4.0
NAV_LOCAL_COSTMAP_HEIGHT_M=4.0
NAV_LOCAL_COSTMAP_RESOLUTION_M=0.06
NAV_LOCAL_COSTMAP_DYNAMIC_DECAY_S=1.0
NAV_LOCAL_COSTMAP_OBSTACLE_RADIUS_M=0.06
NAV_LOCAL_COSTMAP_INFLATION_M=0.08
NAV_LOCAL_COSTMAP_LIDAR_CLEAR_RADIUS_M=0.05
NAV_LOCAL_COSTMAP_MAX_RAYTRACE_M=4.0
NAV_CONTINUOUS_ALLOW_COSTMAP_SOFT_PENALTY=false
```

By default, occupied local-costmap trajectories are hard-rejected. The
soft-penalty switch is only for replay experiments with coarse maps.

`/ugv_nav_status` includes `velocity_control` normalized scoring fields
(`progress_score`, `path_alignment_score`, `clearance_score`,
`gap_alignment_score`, `speed_score`, `smoothness_cost`, rejection counts, and
`final_score`) plus `local_costmap` stats.

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

Replay/status metrics:

```bash
python3 ros2_ws/src/ugv_nav/tools/replay_nav_metrics.py path/to/replay_or_status.jsonl
```

## Notes

- Heading comes from encoder odometry by default.
- Real robot encoder odometry uses `--robot-wheel-radius-m` and
  `--robot-ticks-per-rev`. Ground calibration on the current chassis measured
  `old_config_odom_est_m=0.3872` versus `actual_m=0.1800`, so test this chassis
  with `--robot-ticks-per-rev 2151` or `ROBOT_TICKS_PER_REV=2151`.
- Optional ZED IMU yaw-rate blending exists but stays disabled until axis/sign are verified.
- The chassis footprint is modeled with robot length, width, track width, obstacle inflation, and safety margins.
- The live map is obstacle-focused. Dashboard camera-coverage visualization is available, but camera coverage is not yet a hard planner objective.
