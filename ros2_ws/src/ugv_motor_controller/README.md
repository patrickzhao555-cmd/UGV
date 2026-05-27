# ugv_motor_controller

Clean active motor path for the current drivetrain hardware.

## Hardware Truth

- Four Pololu motors.
- Four encoder channels.
- Two goBILDA speed controllers.
- The left speed controller drives both left motors.
- The right speed controller drives both right motors.

Because there are only two physical controller inputs, the active architecture is
two-side velocity PID. Four-motor independent PID is not supported by this
hardware revision.

## Active Runtime

```text
/ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> CMD V <v_mps> <omega_radps>
  -> Teensy 4.1 side PID firmware
  -> left/right goBILDA speed controllers
```

The bridge is intentionally thin. It does not run motor PID and it does not
generate PWM from Python. It forwards velocity intent, sends STOP on timeout,
syncs Teensy parameters with ACK verification, and publishes encoder/status
topics.

## Active Files

- `ugv_motor_controller/motor_controller_bridge.py`
- `ugv_motor_controller/teensy_side_pid.py`
- `firmware/teensy_4_1_side_pid_controller/teensy_4_1_side_pid_controller.ino`
- `launch/motor_controller.launch.py`

Removed from active runtime:

- ROS-side velocity PID
- previous two-value Teensy bridge firmware
- direct raw motor test node/launch

Those files are recoverable from the backup branch:

```text
backup/pre-cleanup-20260527
```

## Command Contract

The clean bridge accepts only:

```json
{"command_type":"velocity","v_mps":0.20,"omega_radps":0.0}
{"command_type":"stop","mode":"STOP"}
```

The bridge sends:

```text
CMD V <v_mps> <omega_radps>
CMD STOP
CMD PARAM <name> <value>
```

The Teensy publishes:

```text
S,<millis>,<fl_ticks>,<fr_ticks>,<rl_ticks>,<rr_ticks>,...
PARAM,<name>,ok
PARAM,<name>,unknown
PARAMS,<name>=<value>,...
```

## Bench Start

First run with wheels off the ground:

```bash
scripts/run_teensy_side_pid_bench.sh --yes
```

Default motor model:

```text
wheel_radius_m=0.0889
ticks_per_rev=3200
track_width_m=0.6096
```
