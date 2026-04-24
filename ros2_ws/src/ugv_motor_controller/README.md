# ugv_motor_controller

This package contains the last bridge between navigation output and the actual UGV drivetrain.

It includes:

- a ROS 2 bridge node that subscribes to `/ugv_nav_cmd`
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

Default Jetson-side serial port is `/dev/ttyACM0` because the current UGV wiring uses Teensy USB directly to the Nano. Change it in launch if your wiring is different.
