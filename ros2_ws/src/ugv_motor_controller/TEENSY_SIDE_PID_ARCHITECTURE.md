# Teensy Two-Side PID Architecture

## Confirmed Hardware

```text
4 Pololu motors
2 goBILDA speed controllers
4 encoders
1 Teensy 4.1
```

The left goBILDA controller drives both left motors. The right goBILDA
controller drives both right motors. This gives exactly two independent motor
command outputs.

## Control Ownership

```text
Jetson:
  high-level navigation
  publishes v_mps + omega_radps

Teensy:
  receives CMD V v omega
  computes left/right target speed
  reads four encoders
  runs left/right side PID
  writes PWM to two goBILDA controllers
```

## Encoder Usage

```text
left_measured_tps  = average(FL_tps, RL_tps)
right_measured_tps = average(FR_tps, RR_tps)
```

FL/RL and FR/RR mismatch are diagnostics only. The current hardware cannot
independently correct front-vs-rear speed on the same side.

## Firmware Protocol

Jetson to Teensy:

```text
CMD V <v_mps> <omega_radps>
CMD STOP
CMD PARAM <name> <value>
CMD STATUS
```

Teensy to Jetson:

```text
PARAM,<name>,ok
PARAM,<name>,unknown
S,<millis>,<fl_ticks>,<fr_ticks>,<rl_ticks>,<rr_ticks>,...
```

## Initial Calibration

Use physical wheel radius with the equivalent calibrated ticks:

```text
wheel_radius_m = 0.0889
ticks_per_rev = 3200
track_width_m = 0.6096
```

Do not mix `wheel_radius_m=0.0889` with the old effective
`ticks_per_rev=2151`.
