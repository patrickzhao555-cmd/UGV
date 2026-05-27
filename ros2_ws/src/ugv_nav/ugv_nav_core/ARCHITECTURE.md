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
- `pivot_test`: records current heading, adds a relative target angle, and
  pivots until the heading is within deadband and yaw rate is settled.

The heading estimate integrates IMU `angular_velocity.z`. Encoder yaw from
left/right ticks is maintained as a sanity signal and fallback, but the first
controller does not depend on an IMU orientation quaternion.

## Safety Policy

Active test modes publish STOP if:

- `/sensors/nav_frame` is stale.
- `/motor_controller/status` is stale.
- Teensy PID parameters are not synced.
- Motor status reports a fault.
- Obstacle flags or front clearance require stopping.
- The bounded test duration elapses.

## Non-Goals

- No raw PWM from navigation.
- No motor PID on Jetson.
- No four-motor independent PID while the robot has only two goBILDA speed
  controller actuator outputs.
- No full mission navigation inside this test node.
