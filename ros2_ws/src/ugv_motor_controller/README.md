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

## Firmware Protocol

- Jetson to Teensy: `M<left_us>,<right_us>\n`
- Teensy to Jetson: `E<fl>,<fr>,<rl>,<rr>,<millis>\n`

The bridge has ROS-side safety:

- command timeout returns PWM to neutral
- serial reconnect publishes connection state
- normal commands are slew-limited; startup, timeout, and shutdown stops go directly to neutral
