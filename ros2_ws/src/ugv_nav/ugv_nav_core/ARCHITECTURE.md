# UGV Navigation Core Architecture

This package is being split out of `ugv_nav_dual_mode.py` in small behavior-preserving phases. The compatibility entrypoint still owns the CLI, simulation harness, legacy mission paths, and ROS real-mode loop, but competition-control logic now lives in smaller core modules.

## Module Map

- `competition_mission.py`
  - Owns `CompetitionMissionV2`.
  - Chooses the current competition phase, sweep lane/cell, target navigation goal, target loiter goal, and stop policy.
  - Emits mission status consumed by navigation, status JSON, and feedback supervision.

- `closed_loop_controller.py`
  - Owns row following, heading hold, sign multipliers, forward-arc-only limiting, and closed-loop health supervision.
  - Converts the mission sweep context into final `v_mps` and `omega_radps` for normal competition sweep.
  - Round2 clear sweep uses row follower steering as the authority. Round3 can blend local planner omega through `sweep_planner_omega_weight_round3`.

- `velocity_planner.py`
  - Owns the continuous DWA-style local velocity candidate selection and velocity-to-command conversion.
  - The local planner remains a safety and short-horizon feasibility layer. It does not own Round2 clear-row steering when the closed-loop sweep adapter is active.

- `nav_config.py`
  - Owns unit constants, drive-speed helpers, and configuration dataclasses:
    `RobotConfig`, `SensorConfig`, `NavConfig`, and `SimConfig`.
  - Tuned profile values still resolve through `jetson_bringup.sh` and launch arguments before they become `NavConfig` fields.

- `nav_status.py`
  - Owns `/ugv_nav_status` JSON assembly.
  - Status fields are intentionally preserved, including `competition_v2`, `competition_closed_loop`, local costmap debug, odometry deltas, physical stall fields, and velocity/motor-facing debug.

- `sweep_metrics.py`
  - Owns optional CSV logging under `~/.ros/ugv_sweep_metrics/`.
  - Records pose, cross-track error, heading error, closed-loop command, motor target/measured speeds, active cell, and phase.

- `real_mode_runner.py`
  - Phase 1 extraction for real-mode competition helper functions:
    mission-status conversion, motion-policy adaptation, and feedback creation.
  - The large `run_real_mode` loop remains in `ugv_nav_dual_mode.py` for compatibility during this phase and can be moved in a later narrower patch.

## Competition Control Flow

1. `CompetitionMissionV2` selects the current mission goal.
   - Round1 extends the straight goal until landed/complete/stop.
   - Round2 sweeps rows until target is known, then target-nav/target-loiter keeps the UGV moving.
   - Round3 adds obstacle-aware skipping/blocking feedback.

2. `closed_loop_controller.py` applies row following during `competition_v2.phase == "sweep_search"`.
   - Lane follower computes cross-track correction.
   - Heading hold tracks row yaw.
   - Speed scheduling, low-pass filtering, rate limiting, forward-only arc clamp, and divergence monitoring shape the final command.

3. `velocity_planner.py` remains the continuous local planner.
   - It samples `(v, omega)` trajectories, rejects unsafe candidates, and provides speed/safety constraints.
   - During Round2 clear sweep, planner omega weight is `0.0` by default so gap direction does not fight the row follower.

4. The motor controller bridge receives the unchanged `/ugv_nav_cmd` JSON contract.
   - In tuned competition mode, `command_type="velocity"` goes to encoder-based wheel-speed PID.
   - STOP, stale encoder, physical stall, obstacle stop, and emergency behavior remain in their existing safety layers.

## Tuned Baseline

`UGV_PROFILE=round2_clear_tuned` is the current behavior baseline. This refactor must not change:

- tuned profile values,
- `CompetitionMissionV2` mission semantics,
- `/ugv_nav_cmd` schema,
- `/ugv_nav_status` field names,
- motor PID math,
- marker/UAV topic contracts.

Future cleanup should continue moving code out of `ugv_nav_dual_mode.py` without deleting legacy modes until each path has dedicated tests or is explicitly retired.
