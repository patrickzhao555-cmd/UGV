# Jetson Chassis Controller

`ugv_nav_dual_mode.py` is a safe high-level chassis test node. It defaults to
`idle`, which publishes STOP only.

When a test mode is explicitly selected, it still uses one command contract:

```json
{"command_type":"velocity","v_mps":0.20,"omega_radps":0.0}
```

## Modes

- `idle`: STOP only.
- `straight_test`: drive forward while holding the start heading.
- `pivot_test`: profiled tank-drive turn to a bounded relative angle.

The Teensy remains the only motor velocity PID layer. Jetson heading correction
is done by changing `omega_radps`; navigation must not publish raw PWM and must
not run motor PID.

## Pivot Test

The pivot primitive uses `/zed/imu` directly for high-rate gyro yaw integration.
`/sensors/nav_frame` is still required for encoder diagnostics and obstacle
safety. The default ZED IMU publish rate is 100 Hz; the chassis controller runs
at 50 Hz.

Start with small angles on the ground:

```bash
ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  nav_controller_mode:=pivot_test \
  nav_pivot_angle_deg:=30 \
  nav_max_test_duration_s:=6.0 \
  nav_pivot_timeout_s:=4.0
```

Useful status fields are published on `/ugv_nav_status`: `pivot_state`,
`heading_error_rad`, `yaw_rate_radps`, `gyro_bias_radps`,
`encoder_gyro_disagreement_rad`, `slip_detected`, `pivot_retry_count`, and
`pivot_clearance_m`.

For first bringup, test `+15`, `-15`, `+30`, `-30`, `+45`, `-45`, then `+90`
and `-90`. Only test `180` after 90-degree turns are repeatable.
