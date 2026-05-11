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
  - stable pathing contract that navigation should use; includes scan, smoothed IMU, depth obstacle points, encoder ticks, and front clearance

Rule of thumb:

- if you add a new sensor and it only helps perception or debugging, keep the change inside the adapter node and `fusion_node`
- only extend `/sensors/nav_frame` when navigation genuinely needs new information

## 3. Navigation layer

`ugv_nav/ugv_nav_dual_mode.py` should consume:

- `/sensors/nav_frame`
- `/ugv_goal`
- `/ugv/target` as a `std_msgs/String` JSON marker coordinate from ESP/UAV. Coordinates are meters from the field lower-left origin
- `/ugv/field_map` as a legacy optional `std_msgs/String` JSON 15x15 matrix for older bench tests
- `/ugv/marker_detection` as a `geometry_msgs/PointStamped` when camera/CV confirms the marker first
- `/ugv/mission_flag` as a `std_msgs/String` for ESP/UAV state simulation, such as `landing`, `leaving`, or `scanning`

It should not subscribe to raw lidar, raw ZED, or raw encoder topics directly.

That keeps pathing stable even when the hardware layer changes.

## 4. Marker vision layer

`ugv_perception/marker_vision_node.py` is optional and only starts when
`START_MARKER_VISION=true`.

It consumes:

- `/zed/image`
- `/zed/depth`
- `/ugv_nav_status`

and publishes:

- `/ugv/marker_detection`
- `/ugv/marker_vision_debug`

Navigation already treats `/ugv/marker_detection` as the highest-priority target
source and publishes `/ugv/uav_flag` for the UAV handoff.

## 5. Actuation layer

`ugv_motor_controller/motor_controller_bridge.py` should consume:

- `/ugv_nav_cmd`

and publish:

- `/encoder_ticks`
- `/encoder_ticks_stamped`
- `/motor_controller/status`

This is the last bridge between ROS and the Teensy 4.1.

## 6. Competition/debug helper topics

- `/ugv/target`: ESP/UAV marker target coordinate. Preferred payload is `{"type":"ugv_target_v1","x":6.86,"y":6.86,"unit":"m"}` in a lower-left-origin field frame.
- `/ugv/field_map`: legacy optional JSON field map. Matrix cells use `0` unknown/free, `1` known obstacle, `2` UGV start, `3` marker destination. Matrix row `0` is the upper edge of the field and col `0` is the left edge.
- `/ugv/mission_flag`: manual or ESP/UAV mission state. A `landing` flag reduces command speed while nav continues obstacle avoidance.
- `/ugv_nav_status`: navigation pose, phase, active goal, planner, command, encoder, and sensor-hit summary.
- `/ugv/debug_status`: combined one-line status from ZED, fusion, motor, navigation, and UAV flag topics.
- `/zed/status`: ZED depth health summary, including valid depth sample count and p10 depth.
- `/ugv/uav_flag`: target-found handoff flag for ESP/UAV integration.

## How to add a new sensor later

1. Create or update one adapter node for the hardware.
2. Publish a normalized topic with a clear timestamp.
3. Decide whether `fusion_node` needs to use it.
4. Only if nav truly needs the new information, extend `/sensors/nav_frame`.

If a new sensor is optional or experimental, try to keep it out of the nav contract at first.
