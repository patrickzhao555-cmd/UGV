# Clean Navigation Architecture

The old `ugv_nav_dual_mode.py` implementation was intentionally removed on the
cleanup branch. It mixed simulation, manual control, indoor behavior,
competition mission variants, local planning, recovery, row following, and raw
motor fallback in one runtime.

The new navigation stack should be rebuilt around one active contract:

```text
Jetson navigation
  -> /ugv_nav_cmd velocity JSON
  -> motor_controller_bridge
  -> Teensy two-controller/four-encoder side PID
```

## Required Active Modules

The new implementation should be split by responsibility:

- mission state: round/phase/goal selection
- local safety: obstacle and stop policy
- motion control: converts current goal and robot state into `v_mps` and
  `omega_radps`
- command publisher: publishes velocity-only `/ugv_nav_cmd`
- status publisher: exposes nav state and motor-facing intent

## Non-Goals

- No raw PWM from navigation.
- No motor PID on Jetson.
- No four-motor independent PID while the robot has only two goBILDA speed
  controller actuator outputs.
- No legacy mission fallback inside the active runtime.

## Safe Placeholder

`ugv_nav_dual_mode.py` is currently a STOP-only placeholder so launch files can
start safely while the new Jetson navigation runtime is written.
