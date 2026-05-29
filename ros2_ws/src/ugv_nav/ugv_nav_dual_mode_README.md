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
- `mission_sequence`: run relative straight/pivot/wait mission segments.

The Teensy remains the only motor velocity PID layer. Jetson heading correction
is done by changing `omega_radps`; navigation must not publish raw PWM and must
not run motor PID.

## Competition Motion Rule

Mission mode enforces:

```text
v_mps == 0.0
or
abs(v_mps) >= 0.089408
```

This means STOP is allowed, pivot-in-place is allowed as `v_mps=0.0`, and
straight motion is never commanded below 0.2 mph unless explicit debug sub-min
crawl is enabled.

## Mission Sequence

Mission files are JSON or YAML with relative segments:

```json
{
  "mission_id": "course_01",
  "segments": [
    {"type": "straight", "distance_m": 1.0, "speed_mps": 0.15},
    {"type": "pivot", "angle_deg": 90.0},
    {"type": "straight", "distance_m": 1.0, "speed_mps": 0.15}
  ]
}
```

Run it through the competition bringup:

```bash
ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  nav_controller_mode:=mission_sequence \
  nav_mission_file:=/home/bluelule/ugv_project/missions/course_01.json
```

Mission telemetry is written to `~/.ros/ugv_mission_logs` by default. Analyze a
log with:

```bash
python3 tools/analyze_mission_log.py ~/.ros/ugv_mission_logs/<log>.jsonl
```

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
