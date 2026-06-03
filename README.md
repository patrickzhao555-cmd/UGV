# UGV Clean Runtime Branch

This branch is a deliberate cleanup reset.

Backup of the pre-cleanup code:

```text
backup/pre-cleanup-20260527
```

## Hardware Truth

- Four Pololu motors.
- Each Pololu motor has its own quadrature encoder, so four encoder channels
  are available.
- Two goBILDA 1x15A R/C PWM speed controllers.
- Left controller drives both left motors.
- Right controller drives both right motors.
- The goBILDA controllers are actuator inputs only; they do not provide encoder
  feedback.

Therefore the active motor architecture is **two-controller/four-encoder side
PID**, not four-motor independent PID.

## Active Direction

```text
Jetson high-level navigation
  -> /ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> Teensy two-controller/four-encoder side PID firmware
  -> left/right goBILDA speed controllers
```

The Jetson should publish robot intent:

```json
{"command_type":"velocity","v_mps":0.20,"omega_radps":0.0}
{"command_type":"stop","mode":"STOP"}
```

The Teensy should own:

- encoder reading
- left/right side velocity PID
- PWM output to the two goBILDA controllers
- command timeout
- stall/mismatch/fault diagnostics

The same-side encoder streams, FL/RL and FR/RR, are averaged for PID feedback.
Large same-side mismatch is a diagnostic warning/fault; it cannot be actively
corrected because each side shares one physical speed controller.

## What Was Removed

The cleanup removes the old mixed runtime:

- legacy ROS-side motor PID
- legacy raw Teensy bridge firmware
- direct raw motor test node
- old multi-mode navigation implementation
- old nav core modules and their tests
- old tuned round launch wrappers

`ugv_nav_dual_mode.py` is now the safe Jetson chassis controller entrypoint. It
keeps `idle`, `straight_test`, and `pivot_test` for bringup, and adds
`mission_sequence` for relative straight/pivot/wait missions. Navigation still
publishes only velocity/STOP JSON; raw PWM stays out of Jetson navigation.

Formal competition autonomous travel enforces the corrected minimum moving
speed rule:

```text
before official movement: STOP allowed
active travel: abs(v_mps) >= 0.0894, target crawl 0.12 m/s
destination reached or safety/fault/kill: STOP allowed
```

Manual teleop and calibration tests remain debug exceptions.  During formal
competition autonomous movement, normal waiting/replanning/alignment does not
intentionally command zero speed; safety exceptions are logged explicitly.

## Start Clean Runtime

Build the workspace, then:

```bash
cd ros2_ws
./jetson_bringup.sh
```

For motor bench with wheels off the ground:

```bash
cd ..
scripts/run_teensy_side_pid_bench.sh --yes
```

For a dry mission-sequence run, provide a mission JSON file:

```bash
cd ros2_ws
ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  nav_controller_mode:=mission_sequence \
  nav_mission_file:=/path/to/course_01.json
```
