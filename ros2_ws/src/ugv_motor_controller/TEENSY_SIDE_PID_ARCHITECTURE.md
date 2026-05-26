# Teensy Side PID Motor Control Architecture

## Hardware Scope

The current Teensy 4.1 firmware and repo evidence support two independent motor
command outputs:

- `PWM_L` on Teensy pin 34 drives both left-side motors.
- `PWM_R` on Teensy pin 35 drives both right-side motors.

The PCB/firmware does expose four encoder channels: front-left, front-right,
rear-left, and rear-right. Because there are only two motor command outputs, the
controller cannot independently correct front-vs-rear speed on the same side.
Four encoders are used for side-speed feedback and diagnostics, not independent
actuation.

## Control Split

The Jetson remains the high-level chassis controller. It publishes the existing
`/ugv_nav_cmd` JSON contract with `command_type="velocity"`, `v_mps`, and
`omega_radps`.

When `motor_control_location=teensy_pid` or `MOTOR_CONTROL_LOCATION=teensy_pid`
is selected, `motor_controller_bridge.py` does not run ROS-side velocity PID.
It forwards:

```text
CMD V <v_mps> <omega_radps>
CMD STOP
```

The Teensy converts chassis velocity into side targets:

```text
left_mps  = v_mps - omega_radps * track_width / 2
right_mps = v_mps + omega_radps * track_width / 2
```

It then converts m/s to ticks/s and runs two closed-loop controllers:

- left target ticks/s vs average of FL and RL encoder ticks/s -> `PWM_L`
- right target ticks/s vs average of FR and RR encoder ticks/s -> `PWM_R`

The legacy firmware in `firmware/teensy_4_1_motor_bridge/` and ROS-side
`velocity_control.py` stay available as bench fallback.

The Teensy PID path uses physical wheel geometry by default. The UGV has
7 inch diameter wheels, so the initial motor-control calibration is:

```text
wheel_radius_m = 0.0889
ticks_per_rev = 3200
```

The earlier navigation odometry pair `ROBOT_WHEEL_RADIUS_M=0.06` and
`ROBOT_TICKS_PER_REV=2151` was an effective calibration. Its physical-radius
equivalent is about `0.0889 / 3187`. Do not mix `0.0889` with `2151`.

## Firmware Protocol

New firmware:

```text
ros2_ws/src/ugv_motor_controller/firmware/teensy_4_1_side_pid_controller/
```

Commands:

```text
CMD V <v_mps> <omega_radps>
CMD STOP
CMD RAW2 <left_us> <right_us>
CMD PARAM <name> <value>
CMD STATUS
```

Status:

```text
S,<millis>,<fl_ticks>,<fr_ticks>,<rl_ticks>,<rr_ticks>,
  <fl_tps>,<fr_tps>,<rl_tps>,<rr_tps>,
  <left_target_tps>,<right_target_tps>,
  <left_measured_tps>,<right_measured_tps>,
  <left_pwm>,<right_pwm>,
  <left_error>,<right_error>,
  <left_p>,<left_i>,<left_d>,
  <right_p>,<right_i>,<right_d>,
  <fault>
```

The firmware also emits legacy `E<fl>,<fr>,<rl>,<rr>,<millis>` frames by
default so old encoder consumers still work during transition.

On serial connect, the ROS bridge sends `CMD STOP` and then syncs all critical
Teensy parameters:

```text
CMD PARAM track_width_m <track_width_m>
CMD PARAM wheel_radius_m <wheel_radius_m>
CMD PARAM ticks_per_rev <ticks_per_rev>
CMD PARAM kp <velocity_kp>
CMD PARAM ki <velocity_ki>
CMD PARAM kd <velocity_kd>
CMD PARAM command_timeout_ms <command_timeout_s * 1000>
CMD PARAM pwm_min_us <pwm_min_us>
CMD PARAM pwm_neutral_us <pwm_neutral_us>
CMD PARAM pwm_max_us <pwm_max_us>
CMD PARAM pwm_slew_us_per_s <pwm_slew_rate_us_per_s>
CMD PARAM left_motor_sign <teensy_left_motor_sign>
CMD PARAM right_motor_sign <teensy_right_motor_sign>
CMD PARAM fl_encoder_sign <teensy_fl_encoder_sign>
CMD PARAM fr_encoder_sign <teensy_fr_encoder_sign>
CMD PARAM rl_encoder_sign <teensy_rl_encoder_sign>
CMD PARAM rr_encoder_sign <teensy_rr_encoder_sign>
```

The `teensy_*_sign` parameters are independent from legacy bridge inversion.
In Teensy PID mode, the bridge sends logical RAW2 PWM values and the Teensy
applies motor signs.

## Diagnostics

Same-side synchronization cannot be corrected with the current two-output
hardware. The firmware instead detects and reports:

- `left_front_vs_rear_mismatch`
- `right_front_vs_rear_mismatch`
- per-wheel stalls such as `fl_stall`, `rl_stall`, `fr_stall`, `rr_stall`
- encoder sign mismatches such as `fl_sign` or `rr_sign`

If a side target is nonzero, PWM is high, one wheel encoder is near zero, and
the other wheel on that side is moving, the firmware stops and latches a fault.

## Safety

The Teensy starts neutral and immediately neutralizes both side outputs on:

- `CMD STOP`
- command timeout
- actuator/encoder fault
- zero target speed

PID state is reset on STOP, timeout, fault, and target direction reversal.
Outputs are clamped to the configured PWM range and slew-limited.

## Tuning Order

First runs should be with wheels off the ground.

1. Encoder sign test: move each side slowly and confirm encoder ticks increase
   for forward motion. Adjust `fl_encoder_sign`, `fr_encoder_sign`,
   `rl_encoder_sign`, or `rr_encoder_sign` with `CMD PARAM` if needed.
2. Raw direction test: use `CMD RAW2` to confirm left and right side PWM signs
   drive the expected physical direction. Adjust `left_motor_sign` or
   `right_motor_sign` if needed.
3. Left side PID: command a slow forward velocity with the right side disabled
   mechanically or lifted as appropriate, tune `kp`, then add small `ki` only if
   steady-state error remains.
4. Right side PID: repeat for the right side.
5. Straight test: command low `CMD V` forward speed and watch
   `left_measured_tps` and `right_measured_tps`.
6. Pivot test: command `CMD V 0 <omega>`. Pivot left should make left target
   ticks/s negative and right target ticks/s positive. Pivot right should be the
   opposite.

Useful starting parameters:

```text
CMD PARAM kp 0.55
CMD PARAM ki 0.0
CMD PARAM kd 0.0
CMD PARAM ff_us_per_tps 0.0
CMD PARAM pwm_slew_us_per_s 2400
CMD PARAM command_timeout_ms 500
```
