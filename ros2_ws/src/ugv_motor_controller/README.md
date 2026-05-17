# ugv_motor_controller

Motor bridge and drivetrain test package.

For full pull/build/launch instructions, use the consolidated runbooks:

- English: `ros2_ws/JETSON_BRINGUP_CHECKLIST.md`
- Chinese: `ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md`

## Purpose

This package is the last ROS layer before the physical drivetrain.

It provides:

- `motor_controller_bridge`: subscribes to `/ugv_nav_cmd`, writes PWM commands to the Teensy, and publishes encoder feedback
- `motor_direct_test`: bounded direct movement tester for forward, backward, turn, one-side, raw, and sequence tests
- Teensy 4.1 firmware under `firmware/teensy_4_1_motor_bridge/`

## Topics

Subscriptions:

- `/ugv_nav_cmd`

Publications:

- `/encoder_ticks`
- `/encoder_ticks_stamped`
- `/motor_controller/raw_encoders`
- `/motor_controller/connected`
- `/motor_controller/status`

## Normal Launch

The normal stack starts this package through `jetson_bringup.sh`.

Standalone bridge:

```bash
ros2 launch ugv_motor_controller motor_controller.launch.py \
  port:=/dev/ttyACM0 baud:=115200 dry_run:=false \
  invert_left_command:=false invert_right_command:=true
```

Closed-loop wheel-speed PID is available but defaults off. Raw mode remains the
safe fallback and continues to consume `raw_left`/`raw_right`.

Lift the UGV before enabling velocity PID:

```bash
ros2 launch ugv_motor_controller motor_controller.launch.py \
  port:=/dev/ttyACM0 baud:=115200 dry_run:=false \
  invert_left_command:=false invert_right_command:=true \
  velocity_control_enabled:=true prefer_velocity_fields:=true \
  track_width_m:=0.6096 wheel_radius_m:=0.06 ticks_per_rev:=1000 \
  velocity_kp:=0.80 velocity_ki:=0.0 velocity_kd:=0.02 \
  velocity_feedforward_raw_per_mps:=1.35 velocity_max_target_mps:=0.35
```

Velocity mode consumes `/ugv_nav_cmd` JSON fields `command_type=velocity`,
`v_mps`, and `omega_radps`, computes left/right wheel-speed targets, estimates
measured wheel speed from encoder deltas, then outputs PWM using feedforward
plus PID correction.

When the Teensy encoder packet includes controller milliseconds, the bridge uses
that timestamp for wheel-speed `dt`; otherwise it falls back to host monotonic
time. The measured speed path has optional low-pass filtering and a sanity clamp:

```text
velocity_encoder_speed_filter_alpha=0.65
velocity_encoder_speed_max_mps=2.0
```

Ground odometry calibration for the current chassis recommends an effective
`ROBOT_TICKS_PER_REV=2151` for navigation. If velocity PID is used for true
ground speed later, calibrate this bridge's `ticks_per_rev` /
`MOTOR_TICKS_PER_REV` consistently with the nav odometry value.

`/motor_controller/status` publishes `encoder_speed_dt_source`,
`encoder_speed_dt_s`, and `encoder_speed_anomaly` so bad timestamp deltas or
sanity clamps are visible during bench testing.

Safety defaults:

- `velocity_control_enabled=false`
- `prefer_velocity_fields=true`
- `velocity_stale_encoder_timeout_s=0.25`
- `velocity_fallback_to_raw_without_encoder=false`
- `velocity_encoder_speed_filter_alpha=0.65`
- `velocity_encoder_speed_max_mps=2.0`

If encoder speed is missing/stale in velocity mode, the bridge commands neutral
PWM and resets PID integrators unless raw fallback is explicitly enabled.

## Direct Motor Test

Use this with the UGV lifted for the first run. Do not run full navigation at
the same time, because navigation also publishes `/ugv_nav_cmd`.

```bash
ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=forward raw:=0.22 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=false invert_right_command:=true
```

Available motions:

```text
forward, backward, turn_left, turn_right,
left_forward, left_backward, right_forward, right_backward,
raw, sequence
```

If `motion:=forward` makes one side drive backward and the other side drive
forward, fix command inversion on the physically wrong side before tuning
navigation. Only fix encoder inversion after wheel directions are physically
correct but odometry signs are still wrong.

The direct test is open-loop: `duration_s` and `raw` do not command an exact
angle. The summary prints encoder-based `est_ds` and `est_yaw` so you can
measure how much the robot actually moved/turned for a given raw command.

## Firmware Protocol

- Jetson to Teensy: `M<left_us>,<right_us>\n`
- Teensy to Jetson: `E<fl>,<fr>,<rl>,<rr>,<millis>\n`

The bridge has ROS-side safety:

- command timeout returns PWM to neutral
- STOP returns PWM to neutral immediately and resets velocity PID integrators
- serial reconnect publishes connection state
- normal commands are slew-limited; startup, timeout, and shutdown stops go directly to neutral

`/motor_controller/status` includes `control_mode`, target/measured wheel
speeds, velocity errors, PID terms, `last_pwm`, `target_pwm`, and
`command_age_s`.
