# Teensy Two-Controller/Four-Encoder Side PID Architecture

## Confirmed Hardware

```text
4 Pololu 50:1 37D motors
4 Pololu motor quadrature encoders
2 goBILDA 1x15A R/C PWM speed controllers
1 Teensy 4.1
```

The left goBILDA controller drives both left motors. The right goBILDA
controller drives both right motors. This gives exactly two independent motor
command outputs.

The goBILDA controller is a brushed DC speed controller with an R/C PWM input.
It is not an encoder and it does not provide closed-loop feedback. Encoder
feedback comes from the four Pololu motors.

## Control Ownership

```text
Jetson:
  high-level navigation
  publishes v_mps + omega_radps

Teensy:
  receives CMD V v omega
  computes left/right target speed
  reads four encoders
  runs left/right side PID at 50 Hz by default
  writes PWM to two goBILDA controllers
```

## Encoder Usage

```text
left_measured_tps  = average(FL_tps, RL_tps)
right_measured_tps = average(FR_tps, RR_tps)
```

FL/RL and FR/RR mismatch are diagnostics only. The current hardware cannot
independently correct front-vs-rear speed on the same side.

Fault diagnostics include:

- per-wheel near-zero encoder speed while the other wheel on that side moves
- whole-side stall when both same-side encoders stay near zero with high PWM
- same-side encoder mismatch above the configured fault threshold
- impossible encoder jump while in velocity or raw test mode
- encoder sign mismatch during commanded motion, after a persistence timeout

Motor faults latch on the Teensy. A later `CMD V` or `CMD RAW2` does not clear
the fault; send `CMD STOP` first, inspect wiring/status, then restart the test.

## Firmware Protocol

Jetson to Teensy:

```text
CMD V <v_mps> <omega_radps>
CMD STOP
CMD PARAM <name> <value>
CMD STATUS
CMD STATUS_STREAM <0|1>
```

Teensy to Jetson:

```text
PARAM,<name>,ok
PARAM,<name>,unknown
S,<millis>,<fl_ticks>,<fr_ticks>,<rl_ticks>,<rr_ticks>,...
CTRL,status_stream,<on|off>
```

The firmware requires these Arduino/Teensy libraries:

```text
Encoder
QuickPID
Servo
```

`Servo` is normally provided by the Teensy/Arduino core. Install `Encoder` and
`QuickPID` through the Arduino Library Manager before flashing a new Teensy. If
one of these libraries is missing, compilation should fail instead of silently
using a hand-written fallback. The `PARAMS` line reports
`pid_backend=QuickPID`.

Do not install a separate generic Arduino `Servo` library for Teensy 4.1. If
the compiler includes `Documents/Arduino/libraries/Servo/src/Servo.h` and
reports that Servo only supports AVR/SAM/etc. boards, remove or rename that
external `Servo` folder so the Teensy core's Servo library is used.

Startup parameter sync is ACK-gated. The bridge sends `CMD PARAM` for wheel
model, PID gains, PWM limits, motor signs, encoder signs, loop rate, and fault
thresholds. It does not allow velocity commands until every parameter returns
`PARAM,<name>,ok`.

## Initial Calibration

Measured robot geometry:

```text
wheel_diameter_m = 0.165
wheel_radius_m = 0.0825
wheel_width_m = 0.096
robot_length_m = 0.74
robot_total_width_m = 0.54
track_width_m = 0.416
front_sensor_offset_m ~= 0.37
```

Use the physical wheel radius with the Pololu gearbox-output encoder count:

```text
wheel_diameter_m = 0.165
wheel_radius_m = 0.0825
ticks_per_rev = 3200
track_width_m = 0.416
pwm_neutral_us = 1500
pwm_min_us = 1100
pwm_max_us = 1900
control_hz = 50
kp = 0.03
ki = 0.0
kd = 0.0
ff_us_per_tps = 0.02
static_ff_us = 90
static_ff_full_target_tps = 1500
left_ff_us_per_tps = 0.02
right_ff_us_per_tps = 0.02
left_static_ff_us = 90
right_static_ff_us = 90
left_pid_output_limit_us = 180
right_pid_output_limit_us = 180
left_motor_sign = 1
right_motor_sign = -1
fl_encoder_sign = -1
fr_encoder_sign = 1
rl_encoder_sign = -1
rr_encoder_sign = 1
sign_mismatch_target_tps = 100
sign_mismatch_timeout_ms = 250
```

Do not mix the current physical wheel radius with the old effective
`ticks_per_rev=2151` calibration pair.

## Bench Sequence

First run must be wheels off ground.

1. Flash `firmware/teensy_4_1_side_pid_controller/teensy_4_1_side_pid_controller.ino`.
2. Start `scripts/run_teensy_side_pid_bench.sh --yes`.
3. Confirm `/motor_controller/status` reports `teensy_pid_params_synced=true`.
4. Hand-check encoder signs by watching FL/FR/RL/RR ticks.
5. Run `tools/teensy_side_pid_direction_test.py --yes` at low PWM.
6. Run `tools/ugv_motor_closed_loop_calibrate.py --port <teensy-port> --yes`.
7. Run `tools/teensy_side_pid_step_test.py --yes`.
8. Run `tools/teensy_side_pid_pivot_test.py --yes`.

Do not tune IMU heading hold until the closed-loop calibration passes mapping,
straight speed tracking, and differential speed tracking. If commanded target
side speeds change but measured side speeds do not follow, the failure is still
below the navigation layer.
