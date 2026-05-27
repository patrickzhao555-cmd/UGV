# UGV Jetson/Nano 中文运行手册

英文版：[JETSON_BRINGUP_CHECKLIST.md](JETSON_BRINGUP_CHECKLIST.md)

这份手册对应当前干净分支：`cleanup/two-side-pid-runtime`。

硬件事实：

- 四个 Pololu 电机
- 两个 goBILDA speed controller
- 左侧一个 controller 带两个左侧电机
- 右侧一个 controller 带两个右侧电机
- 四路 encoder 存在
- 物理上只有左右两路独立电机控制输出

所以当前成熟方案是两侧 PID，不是四电机独立 PID。

## 1. 拉代码和编译

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

每个新的 ROS terminal 都要先 source：

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
```

## 2. 干净启动

当前 cleaned runtime 会启动：

- 很薄的 motor bridge
- Teensy side PID 参数同步
- 只发布 STOP 的 navigation placeholder

```bash
cd ~/ugv_project/ros2_ws
MOTOR_PORT=/dev/ttyACM0 \
MOTOR_DRY_RUN=false \
MOTOR_TRACK_WIDTH_M=0.425 \
MOTOR_WHEEL_RADIUS_M=0.0825 \
MOTOR_TICKS_PER_REV=3200 \
MOTOR_TEENSY_PID_KP=0.80 \
MOTOR_TEENSY_PID_KI=0.0 \
MOTOR_TEENSY_PID_KD=0.02 \
MOTOR_TEENSY_CONTROL_HZ=100.0 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash \
bash jetson_bringup.sh
```

只做软件检查时：

```bash
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

注意：这个分支的 navigation 现在故意只发 STOP。新的 Jetson high-level nav
还没开始写，所以不要期待它自动跑任务。

## 3. Teensy Side PID Bench

第一次跑一定要架空轮子。

```bash
cd ~/ugv_project
bash scripts/run_teensy_side_pid_bench.sh --yes
```

确认 `/motor_controller/status` 里这些字段正常：

- `control_mode=two_controller_four_encoder_side_pid`
- `motor_hardware=four_pololu_37d_50_1_motors_two_gobilda_1x15a_pwm_controllers`
- `accepted_command_contract=velocity_only`
- `teensy_pid_params_synced=true`
- `track_width_m=0.425`
- `wheel_radius_m=0.0825`
- `ticks_per_rev=3200`

## 4. Bench 工具

```bash
python3 tools/teensy_side_pid_direction_test.py --port /dev/ttyACM0 --yes
python3 tools/teensy_side_pid_step_test.py --port /dev/ttyACM0 --yes
python3 tools/teensy_side_pid_pivot_test.py --port /dev/ttyACM0 --yes
```

这些工具都是短时间测试，并且都需要 `--yes`。

## 5. 常用 Topic

```bash
ros2 topic echo /motor_controller/status --once --full-length
ros2 topic echo /encoder_ticks_stamped --once
ros2 topic echo /ugv_nav_status --once --full-length
ros2 topic echo /ugv/debug_status --full-length
```

重点看 motor 字段：

- `teensy_pid_params_synced`
- `teensy_pid_param_sync_reason`
- `left_target_tps`, `right_target_tps`
- `left_measured_tps`, `right_measured_tps`
- `left_pwm`, `right_pwm`
- `pid_left`, `pid_right`
- `fault`, `fault_reason`

## 6. 安全规则

- 第一次电机测试必须架空轮子。
- 下地测试前必须确认 Teensy 参数 ACK 同步成功。
- Navigation 只发送速度意图。
- Jetson 不跑 motor PID。
- 不要把 raw PWM 重新加回 navigation 主路径。
