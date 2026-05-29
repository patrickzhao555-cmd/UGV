# Jetson Chassis Control Architecture

`ugv_nav_dual_mode.py` is now the safe Jetson chassis test entrypoint. It is not
full mission navigation. Its first job is to verify two high-level behaviors:
hold a straight heading and pivot to a bounded relative angle.

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
- Jetson chassis controller: heading/yaw tests that publish `v_mps` and
  `omega_radps`.
- Future mission navigation: goal selection and behavior sequencing.

## Chassis Test Modes

- `idle`: publishes STOP continuously.
- `straight_test`: records current heading, drives forward, and applies
  `omega_radps` correction from heading error plus gyro damping.
- `pivot_test`: records current heading, adds a relative target angle, then
  runs a profiled pivot primitive with breakaway, rotate, approach, brake,
  settle, and at most one correction retry.

The heading estimate integrates high-rate `/zed/imu` gyro data directly. The
ZED depth/image path can stay at 10 Hz, while IMU defaults to 100 Hz and the
chassis control timer defaults to 50 Hz. Encoder yaw from left/right ticks is
maintained as a sanity/slip signal, but tank-drive pivot angle truth comes from
gyro integration because skid is expected during turns.

Before each active test, the node holds STOP and calibrates gyro bias. If the
robot moves or gyro samples are too noisy during calibration, it keeps holding
STOP and restarts calibration.

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

## Non-Goals

- No raw PWM from navigation.
- No motor PID on Jetson.
- No four-motor independent PID while the robot has only two goBILDA speed
  controller actuator outputs.
- No full mission navigation inside this test node.
