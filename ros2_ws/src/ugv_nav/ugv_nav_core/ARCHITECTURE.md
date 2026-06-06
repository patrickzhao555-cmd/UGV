# Jetson Chassis Control Architecture

`ugv_nav_dual_mode.py` is the safe Jetson chassis controller. Competition
controllers use encoder distance and IMU yaw as local odometry, then
continuously correct heading, distance, and cross-track error before every
command. The older Nav2 field launch, Operation Touchdown mission supervisor,
and Nav2 adapter are retained as a separate legacy/debug stack, not an
automatic competition fallback.

The active command contract remains:

```text
Jetson navigation
  -> /ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> Teensy two-controller/four-encoder side PID
```

## Layering

The split is intentional:

- Teensy: low-level left/right side velocity PID.
- Jetson motor bridge: thin protocol bridge and motor health/status.
- Jetson chassis controller: Challenge 1 straight heading hold, Challenge 2
  align-then-straight tracking, target-path tracking, heading/yaw tests, and
  mission sequencing that publish `v_mps` and `omega_radps`.
- Legacy Nav2 adapter and mission supervisor: optional debug target navigation
  and collision-monitor handoff.

## Chassis Test Modes

- `idle`: publishes STOP continuously.
- `competition_tracker`: takes a target from `/ugv/uav_target` or manual launch
  parameters, integrates encoder distance with IMU yaw, then tracks the local
  target path until it reaches the configured stop radius.
- `challenge2_align_straight`: waits for a target, pivots in place until IMU
  heading settles on the target bearing, then tracks the start-local straight
  line to the marker using encoder odometry and IMU yaw.
- `straight_test`: records current heading, drives forward, and applies
  `omega_radps` correction from heading error plus gyro damping.
- `pivot_test`: records current heading, adds a relative target angle, then
  runs a profiled pivot primitive with breakaway, rotate, approach, brake,
  settle, and at most one correction retry.
- `mission_sequence`: runs a JSON/YAML relative mission with `straight`,
  `pivot`, and `wait` segments.

The heading estimate integrates high-rate `/zed/imu` gyro data directly. The
ZED depth/image path can stay at 10 Hz, while IMU defaults to 100 Hz and the
chassis control timer defaults to 50 Hz. In `competition_tracker`, encoder
average distance supplies actual translation and IMU yaw supplies actual
heading. The competition controllers continuously recompute remaining distance,
cross-track error, heading error, and obstacle state before every command.

Before each active test, the node holds STOP and calibrates gyro bias. If the
robot moves or gyro samples are too noisy during calibration, it keeps holding
STOP and restarts calibration.

## Competition Motion Rule

Formal competition autonomous travel applies the corrected continuous-movement
minimum speed rule:

```text
before official movement: STOP allowed
active competition travel: abs(v_mps) >= 0.0894, target crawl 0.12 m/s
destination reached or safety/fault/kill: STOP allowed
```

Manual teleop and primitive calibration modes are debug exceptions.  During
formal autonomous movement, normal waits, replans, marker search, and terminal
alignment crawl instead of intentionally commanding zero speed.  STOP is still
used for destination reached, kill switch, safety stop, or fault.
Formal autonomous steering through `competition_tracker` and
`challenge2_align_straight` is velocity/omega closed-loop tracking. Legacy Nav2
steering remains rolling-arc constrained by
`ugv_nav2_adapter.py`; with `allow_side_reverse:=false`, pure opposite-side-speed
pivots are not passed to the motor bridge.

Mission files use relative segments:

```json
{
  "mission_id": "course_01",
  "segments": [
    {"type": "straight", "distance_m": 1.0, "speed_mps": 0.15},
    {"type": "pivot", "angle_deg": 90.0},
    {"type": "straight", "distance_m": 1.0, "speed_mps": 0.15}
  ]
}
```

`straight` and `pivot` segments can include an explicit `timeout_s`. If it is
omitted, straight segments get a conservative timeout from distance and legal
speed, while pivot segments use the pivot controller timeout.

## Pivot Primitive

`pivot_test` publishes velocity JSON with `v_mps=0.0`; the motor bridge converts
`omega_radps` into opposite left/right side speeds. The state machine is:

```text
idle -> precheck -> gyro_bias_calibration
     -> breakaway -> rotate -> approach -> stop_brake -> settle -> complete
                                           \-> correction_retry -/
```

Important defaults:

- `nav_pivot_max_omega_radps=0.35`
- `nav_pivot_min_omega_radps=0.16`
- `nav_pivot_breakaway_omega_radps=0.18`
- `nav_pivot_accel_limit_radps2=0.80`
- `nav_pivot_decel_limit_radps2=0.60`
- `nav_pivot_settle_error_rad=0.035`
- `nav_pivot_settle_yaw_rate_radps=0.05`
- `nav_pivot_timeout_s=4.0`

## Safety Policy

Active test modes publish STOP if:

- `/sensors/nav_frame` is stale.
- `/zed/imu` is stale.
- `/motor_controller/status` is stale.
- Teensy PID parameters are not synced.
- Motor status reports a fault.
- Obstacle flags or front clearance require stopping.
- Pivot clearance from the scan is below `nav_pivot_clearance_m`.
- The bounded test duration elapses.

Mission mode classifies safety as `ok`, `degraded`, or `critical`. Degraded
conditions may slow to the legal minimum or pause; critical conditions abort
with STOP.

## Non-Goals

- No raw PWM from navigation.
- No motor PID on Jetson.
- No four-motor independent PID while the robot has only two goBILDA speed
  controller actuator outputs.
- No GPS/global planner in v1 mission mode; it composes relative motion
  primitives only.

## Legacy Nav2 Field Navigation Path

The GPS-denied Nav2 obstacle-avoidance path is now a legacy/debug integration
layer. It does not replace `challenge2_align_straight`, `competition_tracker`,
the tested chassis primitives, or the Teensy PID layer:

```text
Nav2 planner/controller
  -> /cmd_vel_raw
  -> collision_monitor   # stop-only hard safety from /scan/filtered
  -> /cmd_vel
  -> ugv_nav2_adapter
  -> /ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> Teensy side velocity PID
```

The `map` frame is the competition field frame: lower-left origin, `+x` to the
right, `+y` upward, and yaw positive counter-clockwise. `ugv_field_odom_node.py`
publishes `map -> odom -> base_link`, using a manual initial field pose plus
wheel encoder distance and ZED gyro yaw. `/initialpose` can update the manual
map anchor during setup.

The first obstacle-avoidance policy is deliberately conservative:

- LiDAR is the hard collision source for Nav2 costmaps.
- ZED/semantic obstacle points are converted to `/sensors/zed_obstacle_cloud`
  and used as conservative marking observations.
- Collision Monitor is enabled by default for Nav2 ground tests. `/cmd_vel_raw`
  is never a motor command; it is checked first and only `/cmd_vel` reaches
  `ugv_nav2_adapter`.
- If the adapter sees stale localization/sensors, motor faults, near-obstacle
  flags, or emergency front clearance, it publishes STOP regardless of Nav2.
- UAV field targets arrive on `/ugv/uav_target` as `PointStamped` in `map`.
  `ugv_operation_touchdown_mission.py` is enabled by default, validates the
  target, chooses a staging pose inside the destination circle, sends Nav2 to
  staging, and then performs terminal ArUco/coordinate stopping. The direct
  `ugv_uav_goal_bridge.py` remains a debug-only direct Nav2 goal bridge.
- Autonomous Nav2 reverse is blocked by default because the rear LiDAR sector
  is physically obstructed.
- Pure Nav2 spin commands are converted to active forward crawl and then
  rolling-arc limited during formal movement, or blocked before motor output
  when side reversal would be required.

Launch the experimental Nav2 stack with:

```bash
ros2 launch ugv_sensor_sync nav2_field_navigation.launch.py \
  initial_x_m:=1.0 initial_y_m:=1.0 initial_yaw_deg:=0.0 \
  field_width_m:=13.716 field_height_m:=13.716
```

Use `field_width_m` and `field_height_m` as the single source of truth for the
competition field. The launch file rewrites the Nav2 global costmap dimensions
from those values so goal validation and planning share the same `map` bounds.
The default is `15 yd x 15 yd` (`13.716 m x 13.716 m`), and the adapter rejects
translational commands in the boundary margin or predicted to leave the safe
inner field as a final containment gate. Start Nav2 ground tests from a pose
inside the safe inner field, not `(0,0)`.

LiDAR observations use `/scan/filtered`, not raw `/scan`. The filter keeps the
front `250 deg` sector and invalidates the rear `110 deg` sector blocked by the
UGV body, so costmaps and Collision Monitor do not treat the chassis as an
obstacle.

Keep `competition_tracker`, `straight_test`, `pivot_test`, manual teleop, and
the motion test runner as the acceptance tools before enabling any legacy Nav2
goal execution.
