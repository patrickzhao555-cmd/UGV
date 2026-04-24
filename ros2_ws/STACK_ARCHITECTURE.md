# UGV Stack Architecture

The stack is organized into four layers so adding or removing hardware is less painful.

## 1. Device adapter layer

Each hardware source should have one small adapter node that only worries about that device.

Current examples:

- `ugv_sensor_sync/zed_sync_node.py`
- `ugv_sensor_sync/lidar_sync_node.py`
- `ugv_motor_controller/motor_controller_bridge.py` for encoder feedback

Responsibilities:

- talk to the hardware
- normalize timestamps
- publish one clean ROS topic per device

## 2. Sensor fusion layer

`ugv_sensor_sync/fusion_node.py` is the only place that should combine sensor streams.

It publishes two main outputs:

- `/sensors/synced`
  - full debug bundle with raw scan, image, depth, imu, and derived obstacle points
- `/sensors/nav_frame`
  - stable pathing contract that navigation should use

Rule of thumb:

- if you add a new sensor and it only helps perception or debugging, keep the change inside the adapter node and `fusion_node`
- only extend `/sensors/nav_frame` when navigation genuinely needs new information

## 3. Navigation layer

`ugv_nav/ugv_nav_dual_mode.py` should consume:

- `/sensors/nav_frame`
- `/ugv_goal`

It should not subscribe to raw lidar, raw ZED, or raw encoder topics directly.

That keeps pathing stable even when the hardware layer changes.

## 4. Actuation layer

`ugv_motor_controller/motor_controller_bridge.py` should consume:

- `/ugv_nav_cmd`

and publish:

- `/encoder_ticks`
- `/encoder_ticks_stamped`
- `/motor_controller/status`

This is the last bridge between ROS and the Teensy 4.1.

## How to add a new sensor later

1. Create or update one adapter node for the hardware.
2. Publish a normalized topic with a clear timestamp.
3. Decide whether `fusion_node` needs to use it.
4. Only if nav truly needs the new information, extend `/sensors/nav_frame`.

If a new sensor is optional or experimental, try to keep it out of the nav contract at first.
