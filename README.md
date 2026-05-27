# UGV Clean Runtime Branch

This branch is a deliberate cleanup reset.

Backup of the pre-cleanup code:

```text
backup/pre-cleanup-20260527
```

## Hardware Truth

- Four Pololu motors.
- Two goBILDA speed controllers.
- Left controller drives both left motors.
- Right controller drives both right motors.
- Four encoder channels are available for feedback and diagnostics.

Therefore the active motor architecture is **two-side PID**, not four-motor PID.

## Active Direction

```text
Jetson high-level navigation
  -> /ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> Teensy side PID firmware
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

## What Was Removed

The cleanup removes the old mixed runtime:

- legacy ROS-side motor PID
- legacy raw Teensy bridge firmware
- direct raw motor test node
- old multi-mode navigation implementation
- old nav core modules and their tests
- old tuned round launch wrappers

The current `ugv_nav_dual_mode.py` is a STOP-only placeholder. It remains so
ROS launch has a safe entrypoint while the new Jetson high-level navigation code
is built.

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
