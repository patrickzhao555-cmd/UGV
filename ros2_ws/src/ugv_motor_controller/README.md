# ugv_motor_controller

Clean active motor path for the current drivetrain hardware.

## Hardware Truth

- Four Pololu 50:1 37D motors.
- Four Pololu motor encoder channels.
- Two goBILDA 1x15A R/C PWM speed controllers.
- The left speed controller drives both left motors.
- The right speed controller drives both right motors.
- The goBILDA speed controllers do not provide encoder feedback.

Because there are only two physical controller inputs, the active architecture is
two-controller/four-encoder side velocity PID. Four-motor independent PID is not
supported by this hardware revision.

## Active Runtime

```text
/ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> CMD V <v_mps> <omega_radps>
  -> Teensy 4.1 two-controller/four-encoder side PID firmware
  -> left/right goBILDA speed controllers
```

The bridge is intentionally thin. It does not run motor PID and it does not
generate PWM from Python. It forwards velocity intent, sends STOP on timeout,
syncs Teensy parameters with ACK verification, and publishes encoder/status
topics.

The Teensy averages FL/RL encoder speed for the left PID and FR/RR encoder
speed for the right PID. Front-vs-rear mismatch on the same side is a
diagnostic warning/fault only, because the same-side motors share one actuator
output.

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
wheel_radius_m=0.0825
ticks_per_rev=3200
track_width_m=0.416
```

## Asymmetric Turn Calibration

Rolling arc commands use standard differential-drive kinematics:

```text
left_mps  = v_mps - omega_radps * track_width_m / 2
right_mps = v_mps + omega_radps * track_width_m / 2
```

For example, `v=0.30`, `omega=+1.20`, `track_width=0.416` asks the left side
for about `0.05 m/s` and the right side for about `0.55 m/s`. If right arcs
work but left arcs do not, the command path is probably asking the right side
to be the fast outside side and the right side is not physically keeping up.

First inspect `/motor_controller/status` and compare:

```text
requested_left_mps / requested_right_mps
target_left_mps / target_right_mps
measured_left_mps / measured_right_mps
left_pwm / right_pwm
fault / fault_reason
```

Bridge-level compensation does not require reflashing the Teensy. Keep speed
scales at `1.0` unless a bench calibration proves the measured side speed is
systematically wrong:

```bash
ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  motor_right_forward_speed_scale:=1.20
```

The Jetson bridge expects the latest Teensy firmware by default and syncs
low-level PID/feedforward parameters at startup. Flash the matching firmware
before running the ROS bridge; if `teensy_pid_params_synced=false`, the bridge
will hold STOP instead of driving with an unknown controller.

Before changing any left/right-specific feedforward or static feedforward, run
the closed-loop bench gate with the wheels off the ground:

```bash
python3 tools/ugv_motor_closed_loop_calibrate.py \
  --port /dev/serial/by-id/usb-Teensyduino_USB_Serial_19983800-if00 \
  --yes
```

Do not carry over one-sided tuning from a previous run. If straight driving
drifts left, making the right side faster will normally increase the left drift
on a tank drive. Only use side-specific values after the calibration output
shows which side fails to reach target and whether PWM is already near its
limit.

Use `motor_pwm_min_us:=1000` and `motor_pwm_max_us:=2000` only after confirming
the speed controllers are calibrated for the full R/C PWM range.
