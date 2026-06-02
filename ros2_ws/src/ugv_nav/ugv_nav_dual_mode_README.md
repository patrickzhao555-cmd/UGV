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

Before the high-level obstacle avoidance layer is mature, mission straight
segments default to STOP/hold on `near_obstacle` or low front clearance instead
of crawling forward. Override only for controlled debug runs with
`nav_mission_stop_on_degraded_obstacle:=false`.

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

`straight` and `pivot` segments may also include `timeout_s`. If omitted,
straight segments use a conservative timeout derived from distance and commanded
speed; pivot segments use `nav_pivot_timeout_s`.

Before a segment starts, mission mode holds STOP until its required start
snapshot is valid: fresh nav frame, valid encoder start ticks for `straight`,
and known/safe pivot clearance for `pivot`.

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
at 50 Hz. The controller subscribes to `/zed/imu` with sensor-data QoS by
default so it is compatible with the ZED sync publisher.

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

## Reliability Diagnostics

`/ugv_nav_status` also reports `imu_rate_hz`, `imu_max_dt_s`,
`imu_skipped_integrations`, `pivot_clearance_known`, `pivot_state_elapsed_s`,
`pivot_overshoot_rad`, `pivot_final_error_rad`, `stuck_detected`, and
`stuck_recovery_count`. Mission telemetry records active segments at
`nav_mission_telemetry_active_hz` so short turns can be analyzed without raising
the public status rate. Records are buffered and flushed by
`nav_mission_telemetry_flush_period_s` / `nav_mission_telemetry_flush_max_records`
to keep file I/O out of the 50 Hz control path as much as possible.
Telemetry is diagnostic only: file I/O failures are reported through
`telemetry_error` and disable further log writes without affecting STOP or
velocity command publication.

When the motor bridge runs with `motor_dry_run:=true`, mission stuck detection is
bypassed with `stuck_reason=dry_run_bypass`; dry-run validates sequencing and
command generation, not physical wheel or yaw response.

Unknown pivot clearance is unsafe by default. If the LiDAR scan is missing,
empty, or contains no finite ranges, pivot commands hold STOP with
`pivot_clearance_unknown`; use `nav_debug_allow_unknown_pivot_clearance:=true`
only with the wheels off the ground or in a controlled debug run.

For first bringup, test `+15`, `-15`, `+30`, `-30`, `+45`, `-45`, then `+90`
and `-90`. Only test `180` after 90-degree turns are repeatable.

## Nav2 Field Navigation

Full GPS-denied obstacle avoidance is launched separately from the chassis
primitive controller:

```bash
ros2 launch ugv_sensor_sync nav2_field_navigation.launch.py \
  initial_x_m:=1.0 \
  initial_y_m:=1.0 \
  initial_yaw_deg:=0.0 \
  field_width_m:=13.716 \
  field_height_m:=13.716
```

This launch starts the existing motor/sensor bringup with `start_nav:=false`,
then adds:

- `ugv_field_odom_node.py`: publishes `map -> odom -> base_link`, `/odom`, and
  localization status from manual field pose plus encoder/gyro odometry.
- `ugv_nav2_adapter.py`: converts Nav2 `/cmd_vel` to `/ugv_nav_cmd` velocity
  JSON, clamps translational motion to the 0-or-0.2MPH rule, and independently
  STOPs on stale localization/sensors or motor faults.
- `ugv_posearray_to_cloud.py`: converts fused ZED/semantic obstacle PoseArrays
  into `/sensors/zed_obstacle_cloud` for Nav2 costmaps.
- `ugv_field_map_node.py`: publishes the static `/map` occupancy grid used by
  Nav2. The field interior is known free space and the configured boundary
  margin is lethal keepout, so `allow_unknown: false` does not make the whole
  competition field unplannable.
- `ugv_operation_touchdown_mission.py`: the default formal competition mission
  supervisor. It accepts the UAV marker center coordinate, chooses a safe
  staging pose inside the 5 ft destination circle, sends Nav2 to staging, then
  performs conservative ArUco/coordinate terminal stopping.
- `ugv_aruco_marker_node`: standard OpenCV ArUco detector for the 6x6,
  `1 ft x 1 ft`, ID `0-4` competition markers.
- `ugv_uav_goal_bridge.py`: retained as a direct Nav2 goal bridge for debug;
  it is not enabled by default in the formal Operation Touchdown launch.

The LiDAR driver publishes raw `/scan`, `lidar_sync_node.py` republishes
`/scan/synced`, and `lidar_scan_filter_node.py` invalidates the blocked rear
sector before publishing `/scan/filtered`. The default keeps the front `250 deg`
sector and removes the rear `110 deg` sector blocked by the UGV body. Fusion,
Nav2 costmaps, and Collision Monitor all consume `/scan/filtered`.

Collision Monitor is enabled by default in this Nav2 launch. Nav2 is remapped
to publish raw velocity on `/cmd_vel_raw`; Collision Monitor publishes the
checked `/cmd_vel`; the adapter is still the only node that may publish
`/ugv_nav_cmd`. Do not connect `/cmd_vel_raw` directly to the motor bridge.
`start_collision_monitor:=false` is only for controlled dry-run/debug work, not
real obstacle tests.

Before using this mode on the ground, verify the TF tree in RViz and tune robot
footprint plus sensor extrinsics. The launch exposes LiDAR/ZED static transform
args such as `lidar_x_m`, `lidar_y_m`, `lidar_yaw_rad`, `zed_x_m`, and
`zed_yaw_rad`; the default x/y/yaw values are placeholders until measured on
the real robot.
Set competition field size only through launch args such as `field_width_m` and
`field_height_m`; the launch file rewrites the global costmap dimensions and
the static field map publisher to match the same field coordinates used by the
UAV goal bridge. The default field is `15 yd x 15 yd`, i.e.
`13.716 m x 13.716 m`. The Nav2 adapter also blocks autonomous translational
commands inside the boundary margin and predicted near-future exits while still
allowing STOP and in-place turns. Do not use an initial pose at `(0,0)` for
ground Nav2; that pose is in the boundary margin and should correctly hold
STOP.

Autonomous Nav2 reverse is disabled by default because the rear LiDAR sector is
blocked by the UGV body. Manual WASD teleop may still reverse under direct
human supervision.

UAV targets are strict by default: `/ugv/uav_target` must be a `PointStamped`
in the `map` frame, in meters unless `uav_target_units:=yards` is set, and a
fresh global costmap must show the target cell as known free space.
The optional `ugv_uav_target_receiver.py` is the shared input adapter for both
ESP and terminal testing. It parses line-based meter coordinates such as
`5.0 7.0`, `5.0,7.0`, `TARGET,5.0,7.0`, `TARGET,seq,5.0,7.0`, or JSON like
`{"x_m":5.0,"y_m":7.0,"seq":12}`, then publishes the same `/ugv/uav_target`
topic. Start it from the Nav2 launch with:

```bash
python3 ros2_ws/src/ugv_nav/ugv_uav_target_receiver.py --ros-args \
  -p input_mode:=terminal \
  -p target_units:=meters
```

For quick one-shot terminal tests, use the standalone publisher:

```bash
python3 tools/send_uav_target.py --x 5.0 --y 7.0
```

It publishes a `PointStamped` in the `map` frame on `/ugv/uav_target` and does
not publish motor commands. Use `--repeat-hz` and `--count` if you want to send
the same coordinate multiple times while checking the mission supervisor.

For ESP line input use:

```bash
ros2 launch ugv_sensor_sync nav2_field_navigation.launch.py \
  start_uav_target_receiver:=true \
  uav_target_input_mode:=serial \
  uav_esp_serial_port:=/dev/ttyUSB1 \
  uav_esp_serial_baud:=115200
```

Checksum suffixes of the form `TARGET,seq,x,y*XX` are supported; set
`uav_esp_require_checksum:=true` once the ESP firmware emits checksums.

For Operation Touchdown, the UAV target is treated as the center of the
destination ArUco marker. The marker is not used as the first global Nav2 goal:
the mission supervisor selects a nearby staging pose instead, so the UGV stops
inside the rulebook's 5 ft destination radius rather than blindly driving over
the flat ground marker. The ArUco detector requires ZED image publishing and
`/zed/left/camera_info`; the Nav2 launch enables `zed_publish_image:=true` by
default. After reaching staging, the supervisor briefly holds STOP to give the
ZED detector a visual confirmation window before falling back to the UAV
coordinate. Coordinate arrival is a hard terminal condition: once localization
places the UGV inside the destination circle, the mission stops even if a fresh
ArUco detection would otherwise suggest driving closer to the marker center.
If the marker is confirmed at that moment, the reason is
`destination_reached_coordinate_marker_confirmed`; otherwise it is
`destination_reached_by_coordinate_no_marker_visual`.
ArUco map projection uses the full camera-to-map TF transform, including ZED
height/pitch/roll. Measure the ZED extrinsics before trusting visual terminal
approach; the yaw-only planar fallback is disabled by default and is only for
debug. A fresh ArUco detection must also agree with the UAV target within
`marker_target_gate_radius_m` before the mission supervisor accepts it.

Before first autonomous motion, run a no-motion check with `motor_dry_run:=true`
and a safe initial pose away from the field margin. Confirm `/scan/filtered` is
near the LiDAR rate and the rear body blockage is removed, `/zed/left/camera_info`
and `/ugv/aruco_detection` exist, `/ugv_nav2_adapter/status` reports boundary
OK, Collision Monitor is active, and RViz shows `map`, `base_link`, LiDAR scan,
ZED obstacles, and field origin aligned with the real robot.

An explicit `/ugv/kill_switch` (`std_msgs/Bool`) is part of the autonomous
safety chain. Publishing `true` causes the Nav2 adapter and mission supervisor
to hold STOP with reason `kill_switch`; publishing `false` releases the latched
software stop, while the hardware/hand-controller kill switch remains the final
safety authority.

## Manual Teleop

For controlled bench or field checks, run the normal bringup and use the
standalone WASD teleop publisher in another terminal. Do not run it while
`ugv_chassis_controller` is also publishing `/ugv_nav_cmd`; either launch only
the motor bridge or start competition bringup with `start_nav:=false`.

```bash
cd ~/ugv_project
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch ugv_motor_controller motor_controller.launch.py
```

Then, in a second terminal:

```bash
cd ~/ugv_project
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 tools/ugv_manual_teleop.py
```

Controls:

- `W`: forward while held.
- `S`: reverse while held.
- `A`: pivot left in place while held.
- `D`: pivot right in place while held.
- `Space` or `X`: STOP.
- `Q` or `Esc`: quit after sending STOP.

The tool only publishes velocity/STOP JSON on `/ugv_nav_cmd`; it does not send
raw PWM or bypass the motor bridge. On Linux, `--input-backend auto` first tries
`/dev/input/event*` so real key press/release events can be used. If input-device
permissions are unavailable, it falls back to terminal auto-repeat with
`--deadman-timeout-s`. For a specific keyboard device:

```bash
python3 tools/ugv_manual_teleop.py --input-backend evdev --keyboard-device /dev/input/event3
```

While a motion key is active, velocity commands are republished at
`--publish-hz` so the motor bridge command timeout stays refreshed. If another
publisher is detected on `/ugv_nav_cmd`, the teleop tool sends STOP and exits to
avoid command fights. Runtime speed changes are clamped by
`--max-forward-mps`, `--max-reverse-mps`, and `--max-turn-radps`.
