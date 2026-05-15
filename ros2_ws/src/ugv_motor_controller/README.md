# ugv_motor_controller

This package contains the last bridge between navigation output and the actual UGV drivetrain.

It includes:

- a ROS 2 bridge node that subscribes to `/ugv_nav_cmd`
- a bounded direct motor test tool for drivetrain direction checks
- serial communication to the Teensy motor controller
- timestamped encoder aggregation from four wheels into left/right side ticks
- the Teensy 4.1 firmware used on the MCU side

## ROS topics

- subscribes to `/ugv_nav_cmd`
- publishes `/encoder_ticks`
- publishes `/encoder_ticks_stamped`
- publishes `/motor_controller/raw_encoders`
- publishes `/motor_controller/connected`
- publishes `/motor_controller/status`

## Launch

```bash
ros2 launch ugv_motor_controller motor_controller.launch.py
```

Useful overrides:

```bash
ros2 launch ugv_motor_controller motor_controller.launch.py \
  port:=/dev/ttyACM0 \
  baud:=115200 \
  raw_command_scale_us:=900.0 \
  pwm_slew_rate_us_per_s:=2400.0 \
  invert_left_command:=false \
  invert_right_command:=false
```

## Firmware

Firmware file:

- `firmware/teensy_4_1_motor_bridge/teensy_4_1_motor_bridge.ino`

Protocol:

- Jetson -> Teensy: `M<left_us>,<right_us>\n`
- Teensy -> Jetson: `E<fl>,<fr>,<rl>,<rr>,<millis>\n`
- USB `Serial` and UART `Serial1` both support the same protocol, so the robot can run over `/dev/ttyACM0` now and still keep `Serial1` available later

The bridge keeps a ROS-side failsafe too:

- if `/ugv_nav_cmd` stops arriving, it sends neutral PWM
- if Teensy serial drops, it reconnects and republishes connection status
- normal navigation commands are slew-limited by `pwm_slew_rate_us_per_s` so tank-drive direction changes are less abrupt; timeout/startup/shutdown stops still go straight to neutral

Default Jetson-side serial port is `/dev/ttyACM0` because the current UGV wiring uses Teensy USB directly to the Nano. Change it in launch if your wiring is different.

## Direct drivetrain tests

Use this when you only want to test motor direction, command inversion, and
encoder signs. Do not run the navigation bringup at the same time, because nav
also publishes `/ugv_nav_cmd`.

One-command launch that starts the motor bridge and runs a short test:

```bash
ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=forward raw:=0.22 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=true invert_right_command:=true
```

Available motions:

```text
forward
backward
turn_left
turn_right
left_forward
left_backward
right_forward
right_backward
raw
sequence
```

For `raw`, pass the two side commands directly:

```bash
ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=raw raw_left:=0.20 raw_right:=-0.20 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=true invert_right_command:=true
```

The tester publishes bounded raw commands for at most 3 seconds, then sends
`STOP` repeatedly. It prints the final encoder delta:

```text
encoder_delta left/right=..., raw fl/fr/rl/rr=...
```

Lift the UGV for the first run. If `motion:=forward` physically drives backward,
flip both command inversion flags together. If a physically forward side makes
that side's encoder ticks decrease, flip that side's encoder inversion flag.
Use `dry_run:=true` first if you only want to verify ROS/serial plumbing.

If the bridge is already running by itself, you can run only the test publisher:

```bash
ros2 run ugv_motor_controller motor_direct_test --ros-args \
  -p motion:=turn_left -p raw:=0.22 -p duration_s:=0.8 -p yes:=true
```
