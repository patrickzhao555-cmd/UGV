# UGV Operation Touchdown Runtime

This repository contains the current competition runtime for the UGV:

- Teensy 4.1 two-controller/four-encoder side PID motor firmware
- ROS 2 motor bridge and sensor sync stack
- Challenge 1 landing-platform controller
- Challenge 2 IMU closed-loop forward-arc alignment and straight drive
- Challenge 3 large-radius corridor bypass controller
- UAV/ESP target packet receiver with manual target fallback

## Hardware Truth

The active drivetrain is:

```text
Jetson high-level navigation
  -> /ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> Teensy two-controller/four-encoder side PID firmware
  -> left/right goBILDA speed controllers
  -> four Pololu motors with four encoder channels
```

Hardware summary: four Pololu motors, four quadrature encoders, and Two goBILDA 1x15A R/C PWM speed controllers. The left controller drives both left motors, and the right controller drives both right motors.

Navigation publishes only velocity/STOP intent. Raw PWM stays on the Teensy.

## Build On Jetson

```bash
cd ~/ugv_project
git pull --ff-only origin main

cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

source ~/ugv_project/ros2_ws/install/setup.bash
```

Default motor port used in the field:

```text
/dev/serial/by-id/usb-Teensyduino_USB_Serial_19983800-if00
```

## Challenge 1

Challenge 1 drives the landing platform path and listens for UAV lifecycle
events on:

```text
/ugv/uav_launched
/ugv/uav_landed
```

Launch Challenge 1:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash

ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  nav_controller_mode:=challenge1_landing_platform \
  nav_challenge1_speed_mps:=0.24 \
  nav_challenge1_post_landing_s:=40.0 \
  start_zed:=true \
  start_lidar:=false \
  start_lidar_filter:=false \
  start_fusion:=false \
  nav_debug_ignore_nav_frame:=true \
  motor_port:=/dev/serial/by-id/usb-Teensyduino_USB_Serial_19983800-if00
```

For bench or field testing, publish the UAV landed event manually:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/send_challenge1_event.py --event landed --count 3
```

Timed Challenge 1 fallback, no UAV landed flag required:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash

ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  nav_controller_mode:=challenge1_landing_platform \
  nav_challenge1_speed_mps:=0.24 \
  nav_challenge1_run_duration_s:=80.0 \
  nav_challenge1_timeout_s:=100.0 \
  start_zed:=true \
  start_lidar:=false \
  start_lidar_filter:=false \
  start_fusion:=false \
  nav_debug_ignore_nav_frame:=true \
  motor_port:=/dev/serial/by-id/usb-Teensyduino_USB_Serial_19983800-if00
```

Set `nav_challenge1_run_duration_s:=0.0` or omit it to use the original UAV
landed flag behavior.

## Challenge 2

Challenge 2 requires the UGV start pose. The helper prompts for:

```text
start x
start y
heading yaw deg, field +x = 0, left turn positive
```

Launch Challenge 2 with manual target fallback:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/run_challenge2_prompt.py
```

Then send a target manually from another terminal:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/send_uav_target.py --x 4.0 --y 2.0 --count 3
```

Launch Challenge 2 with the UGV ESP target receiver enabled:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/run_challenge2_prompt.py --esp-target --uav-esp-port /dev/ttyUSB1
```

The UAV/ESP target packet is a fixed 14-byte little-endian binary payload:

```python
struct.pack("<BiBff", msg_type, seqNum, status_code, target_x, target_y)
```

Required values:

```text
packet length = 14 bytes
msg_type = 1
status_code = 1
target_x/target_y are meters
ESP-NOW channel = 6 on the ESP firmware side
```

The UGV ESP should verify the UAV ESP source MAC and forward the exact 14 bytes
to the Jetson serial port. The Jetson node in
`ros2_ws/src/ugv_sensor_sync/ugv_sensor_sync_nodes/uwb_node.py` publishes valid
packets to `/ugv/uav_target`.

## Challenge 3

Challenge 3 also requires the UGV start pose. The helper prompts for:

```text
start x
start y
heading yaw deg, field +x = 0, left turn positive
```

Launch Challenge 3 with manual target fallback:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/run_challenge3_prompt.py
```

Then send the UAV/marker target manually from another terminal:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/send_uav_target.py --x 6.0 --y 3.0 --count 3
```

Launch Challenge 3 with the UGV ESP target receiver enabled:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/run_challenge3_prompt.py --esp-target --uav-esp-port /dev/ttyUSB1
```

Challenge 3 runs the standalone corridor bypass controller in
`ros2_ws/src/ugv_nav/ugv_challenge3_corridor.py`. It uses encoder + IMU
dead-reckoning, follows the start-to-target baseline, and avoids obstacles by
early lane changes instead of pivot turns.

Default Challenge 3 obstacle settings:

```text
LiDAR filtered forward FOV = 230 deg
LiDAR cluster gate = at least 3 adjacent beams, max 0.35 m gap
lane offsets = 0.0, +1.6, -1.6, +2.2, -2.2 m
destination stop radius = 5 ft minus 0.20 m buffer
```

## Useful Debug Commands

Watch motor status:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash
python3 tools/ugv_motor_status_watch.py --hz 5
```

Motor-only turn envelope check:

```bash
cd ~/ugv_project
source ~/ugv_project/ros2_ws/install/setup.bash

ros2 launch ugv_motor_controller motor_controller.launch.py \
  port:=/dev/serial/by-id/usb-Teensyduino_USB_Serial_19983800-if00

python3 tools/ugv_velocity_burst.py --v-mps 1.70 --omega-radps 7.8 --duration-s 25.0 --yes
```

Run the local test suite:

```bash
python -m pytest tests -q
```
