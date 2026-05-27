# UGV Stack Architecture

This branch is a clean runtime base for the confirmed drivetrain hardware:
four Pololu encoder motors, two goBILDA R/C PWM speed controllers, four
encoder channels, and only two independent motor command outputs.

## 1. Sensor Adapters

Hardware adapter nodes should stay small. They talk to one device, normalize
timestamps, and publish one clean ROS topic per device.

Current examples:

- `ugv_sensor_sync/zed_sync_node.py`
- `ugv_sensor_sync/lidar_sync_node.py`
- `ugv_motor_controller/motor_controller_bridge.py`

## 2. Sensor Fusion

`ugv_sensor_sync/fusion_node.py` is the place where sensor streams are merged.
Navigation should consume `/sensors/nav_frame` instead of raw LiDAR, raw ZED,
or raw encoder topics.

The optional perception nodes can add semantic and marker information, but
they should not change the drivetrain contract.

## 3. Navigation

`ugv_nav/ugv_nav_dual_mode.py` is currently a safe clean-slate placeholder. It
publishes STOP only while the new Jetson high-level navigation stack is being
designed.

The future active navigation contract is velocity-only JSON on `/ugv_nav_cmd`:

```json
{"command_type":"velocity","v_mps":0.20,"omega_radps":0.35}
```

Navigation must not publish raw PWM and must not run motor PID. Its job is
high-level pathing, obstacle avoidance, mission logic, and velocity intent.

## 4. Actuation

`ugv_motor_controller/motor_controller_bridge.py` is intentionally thin.

It consumes:

- `/ugv_nav_cmd`

It publishes:

- `/encoder_ticks`
- `/encoder_ticks_stamped`
- `/motor_controller/raw_encoders`
- `/motor_controller/connected`
- `/motor_controller/status`

It forwards velocity intent to the Teensy:

```text
CMD V <v_mps> <omega_radps>
CMD STOP
```

The Teensy firmware is the real motor controller. It owns:

- v/omega to left/right target conversion
- four encoder reads
- left/right measured speed from FL/RL and FR/RR averages
- two PID loops, one per goBILDA speed controller
- PWM output to the two speed controllers
- encoder mismatch and stall diagnostics
- timeout and startup neutral safety

The goBILDA controllers do not provide encoder feedback. Encoder feedback comes
from the four Pololu motors. Four independent motor PID is not possible on the
current controller setup, because there are only two physical command outputs.
Same-side synchronization is diagnostic only.

## 5. Debug Topics

- `/ugv_nav_status`: current navigation placeholder status, later new nav status
- `/motor_controller/status`: Teensy side PID bridge status and PID debug
- `/encoder_ticks_stamped`: four encoder tick values with controller millis
- `/sensors/synced_summary`: fusion and sensor-health summary
- `/ugv/debug_status`: combined one-line debug summary

## Runbooks

Day-to-day clean-runtime commands live in:

- `ros2_ws/JETSON_BRINGUP_CHECKLIST.md`
- `ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md`
