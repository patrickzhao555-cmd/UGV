# UGV Jetson/Nano Runbook

Chinese version: [JETSON_BRINGUP_CHECKLIST_ZH.md](JETSON_BRINGUP_CHECKLIST_ZH.md)

This runbook matches the clean two-side PID runtime branch.

Hardware truth:

- Four Pololu motors
- Four Pololu motor encoder channels
- Two goBILDA R/C PWM speed controllers
- One speed controller drives the left side
- One speed controller drives the right side
- Encoder feedback comes from the motors, not from the goBILDA controllers
- Only two independent motor command outputs exist

## 1. Pull And Build

```bash
cd ~/ugv_project
git fetch origin
git checkout cleanup/two-side-pid-runtime
git pull --ff-only origin cleanup/two-side-pid-runtime

cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Every new ROS terminal needs the same overlays:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
```

## 2. Clean Bringup

The cleaned runtime starts:

- the thin motor bridge
- the Teensy side PID parameter sync
- a STOP-only navigation placeholder

```bash
cd ~/ugv_project/ros2_ws
MOTOR_PORT=/dev/ttyACM0 \
MOTOR_DRY_RUN=false \
MOTOR_TRACK_WIDTH_M=0.416 \
MOTOR_WHEEL_RADIUS_M=0.0825 \
MOTOR_TICKS_PER_REV=3200 \
MOTOR_TEENSY_PID_KP=0.05 \
MOTOR_TEENSY_PID_KI=0.0 \
MOTOR_TEENSY_PID_KD=0.0 \
MOTOR_TEENSY_PID_FF_US_PER_TPS=0.04 \
MOTOR_TEENSY_PID_STATIC_FF_US=170.0 \
MOTOR_TEENSY_CONTROL_HZ=50.0 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash \
bash jetson_bringup.sh
```

For software-only checks:

```bash
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

The navigation placeholder publishes STOP only. Do not expect autonomous
navigation on this branch until the new high-level Jetson nav is written.

## 3. Teensy Side PID Bench

Use this before any ground run. The first run must be wheels off ground.

```bash
cd ~/ugv_project
bash scripts/run_teensy_side_pid_bench.sh --yes
```

The bridge should report:

- `control_mode=two_controller_four_encoder_side_pid`
- `motor_hardware=four_pololu_37d_50_1_motors_two_gobilda_1x15a_pwm_controllers`
- `accepted_command_contract=velocity_only`
- `teensy_pid_params_synced=true`
- `track_width_m=0.416`
- `wheel_radius_m=0.0825`
- `ticks_per_rev=3200`

## 4. Bench Tools

The active bench tools are:

```bash
python3 tools/teensy_side_pid_direction_test.py --port /dev/ttyACM0 --yes
python3 tools/teensy_side_pid_step_test.py --port /dev/ttyACM0 --yes
python3 tools/teensy_side_pid_pivot_test.py --port /dev/ttyACM0 --yes
```

All tools are bounded duration tests and require `--yes`.

## 5. Useful Topics

```bash
ros2 topic echo /motor_controller/status --once --full-length
ros2 topic echo /encoder_ticks_stamped --once
ros2 topic echo /ugv_nav_status --once --full-length
ros2 topic echo /ugv/debug_status --full-length
```

Important motor fields:

- `teensy_pid_params_synced`
- `teensy_pid_param_sync_reason`
- `left_target_tps`, `right_target_tps`
- `left_measured_tps`, `right_measured_tps`
- `left_pwm`, `right_pwm`
- `pid_left`, `pid_right`
- `fault`, `fault_reason`

## 6. Safety Rules

- First motor test: wheels off ground.
- Confirm Teensy parameter ACK sync before ground testing.
- Navigation should only send velocity intent.
- Jetson should not run motor PID.
- Do not reintroduce raw PWM as a navigation command path.
