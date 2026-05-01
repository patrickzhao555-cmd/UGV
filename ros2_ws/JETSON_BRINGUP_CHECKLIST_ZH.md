# Jetson 启动和 Bench Test 指南

English version: [JETSON_BRINGUP_CHECKLIST.md](JETSON_BRINGUP_CHECKLIST.md)

这份文档假设 repo 在 `~/ugv_project`，ROS 2 workspace 在
`~/ugv_project/ros2_ws`。

停止整套 stack：在 launch terminal 按 `Ctrl+C`。

## 1. 拉取最新代码

```bash
cd ~/ugv_project
git checkout feature/motor-sync-nav-bringup
git pull --ff-only origin feature/motor-sync-nav-bringup
```

确认 `ros2_ws` 属于正确 repo：

```bash
cd ~/ugv_project/ros2_ws
git rev-parse --show-toplevel
```

预期输出：

```text
/home/bluelule/ugv_project
```

## 2. Build

每次 pull 新代码，或者改了 Python/launch 文件之后，运行：

```bash
cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每开一个新 terminal，都先 source：

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
```

## 3. 一条命令启动整套 stack

主入口：

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

如果设备 port 变了：

```bash
LIDAR_PORT=/dev/ttyUSB0 MOTOR_PORT=/dev/ttyACM0 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

如果某一边 motor 或 encoder 方向反了：

```bash
INVERT_LEFT_COMMAND=true INVERT_LEFT_ENCODER=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## 4. 推荐 Bench Mode

UGV 架起来测试时推荐用这个模式。它会启动整套 stack、打开 dry-run motor
输出、启动 debug topics，并自动发布一个测试目标点。

```bash
cd ~/ugv_project/ros2_ws
BENCH_TEST=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

修改自动发布的 bench goal，单位是 meter：

```bash
BENCH_TEST=true BENCH_GOAL_X_M=6.86 BENCH_GOAL_Y_M=6.86 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

正常现象：

- launch terminal 显示 `DRY RUN motor command`，不是 `Sent motor command`
- `/ugv/debug_status` 显示 `phase=manual` 和 `cmd=...`
- dry-run 下 pose 可能不动，因为没有真的给 motor 写 PWM

## 5. 手动 Dry-Run 模式

如果想先 launch，再手动发送 coordinate，用这个。motor 不会收到 PWM。

Terminal 1：

```bash
cd ~/ugv_project/ros2_ws
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Terminal 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash

ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 12.2, y: 12.0, z: 0.0}}"
```

坐标单位是 meter。当前场地模型是 15 yard x 15 yard，所以中心点约为
`(6.86, 6.86)` meter。

## 6. 架车真实 Motor Run

只在 UGV 安全架起来时使用。这个会真的给 motor controller 发 PWM。

Terminal 1：

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Terminal 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash

ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 12.2, y: 12.0, z: 0.0}}"
```

检查 motor bridge：

```bash
ros2 topic echo /motor_controller/status --once --full-length
```

`TURN_LEFT` 时，左 PWM 应低于 neutral，右 PWM 应高于 neutral，比如
`1230, 1770`。

## 7. 放地上实跑

只在 mechanical team 确认平台安全、周围区域清空之后使用。

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

另开 terminal 发布目标点：

```bash
ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 6.86, y: 6.86, z: 0.0}}"
```

建议同时开 debug：

```bash
ros2 topic echo /ugv/debug_status --full-length
```

## 8. Competition Mode

Competition mode 支持从四个角落开始。如果还不知道 final target，UGV 会先去场地中心。
收到 ESP/UAV map 或 camera marker detection 后，会切换到最终目标点。

可选角落：

- `lower_left`
- `lower_right`
- `upper_left`
- `upper_right`

带 mock ESP/UAV map 的 dry-run competition：

```bash
cd ~/ugv_project/ros2_ws
COMPETITION_MODE=true START_CORNER=lower_left START_MOCK_FIELD_MAP=true \
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

修改 mock marker cell：

```bash
COMPETITION_MODE=true START_CORNER=upper_right START_MOCK_FIELD_MAP=true \
MOCK_MARKER_CELL=7,7 MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

不使用 mock map 的 competition-style run：

```bash
COMPETITION_MODE=true START_CORNER=lower_left \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## 9. 手动发送 Field Map

ESP/UAV map topic 是 `/ugv/field_map`，message type 是 `std_msgs/String`。

Cell 约定：

- `0`: unknown 或 free
- `1`: 已知 obstacle
- `2`: UGV start
- `3`: marker destination

Matrix 的 row `0` 是场地上方/北边，col `0` 是左边/西边。

例子：

```bash
ros2 topic pub --once /ugv/field_map std_msgs/msg/String \
"{data: '{\"type\":\"ugv_field_map_v1\",\"source\":\"manual_test\",\"size\":15,\"cell_size_yard\":1.0,\"matrix\":[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,1,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,3,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,1,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,1,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[2,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]}'}"
```

## 10. 模拟 Camera Marker Detection

如果 CV 比 ESP/UAV map 更早找到 marker，可以发布：

```bash
ros2 topic pub --once /ugv/marker_detection geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 6.86, y: 6.86, z: 0.0}}"
```

UGV 应该发布 target-found flag：

```bash
ros2 topic echo /ugv/uav_flag --once --full-length
```

## 11. Debug Topics

在已经 source 的 terminal 里运行：

```bash
ros2 topic echo /ugv/debug_status --full-length
```

```bash
ros2 topic echo /ugv_nav_status --full-length
```

```bash
ros2 topic echo /sensors/synced_summary --once --full-length
```

```bash
ros2 topic echo /zed/status --once --full-length
```

```bash
ros2 topic echo /encoder_ticks_stamped --once
```

```bash
ros2 topic echo /motor_controller/status --once --full-length
```

重要字段：

- `zed_valid` 或 `valid_depth_samples`: ZED depth 是否有有效深度点
- `min_depth_range_m`: ZED ROI 内最近深度
- `front_lidar_range_m`: LiDAR 前方扇区最近点
- `front_clearance_m`: front LiDAR 和 ZED depth clearance 的较小值
- `encoder_available`: fusion 是否拿到新鲜 encoder ticks
- `cmd`: 当前 navigation command
- `pose_m`: encoder odometry 估计 pose

## 12. Bench Test 流程

### ZED Depth 遮挡测试

启动 bench mode：

```bash
BENCH_TEST=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

把盒子放在 ZED camera 正前方，然后看：

```bash
ros2 topic echo /sensors/synced_summary --once --full-length
```

预期：

- 盒子越近，`min_depth_range_m` 越小
- 如果 ZED 看到的是最近障碍物，`front_clearance_m` 会跟着 `min_depth_range_m` 变小
- `valid_depth_samples` 保持非零

### LiDAR 方向测试

启动 bench mode，把盒子分别放在 LiDAR 前、左、右、后：

```bash
ros2 topic echo /sensors/synced_summary --once --full-length
```

预期：

- 只有盒子在真实车头方向时，`front_lidar_range_m` 才明显变小
- `min_lidar_range_m` 是 360 度最近点，所以任意方向近物都可能让它变小

如果盒子放在车头前方但 `front_lidar_range_m` 不变，说明 LiDAR 0 度方向可能还没对准车头。

### Encoder 方向测试

UGV 架起来，使用真实 motor output：

```bash
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

发布 goal 后看 encoder：

```bash
ros2 topic echo /encoder_ticks_stamped
```

预期：

- forward 时左右 averaged ticks 同方向变化
- turning 时左右 averaged ticks 反方向变化

如果符号不对，用：

```bash
INVERT_LEFT_ENCODER=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

或者：

```bash
INVERT_RIGHT_ENCODER=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

### Motor Command Mapping 测试

不启动 nav，只手动发 motor command：

```bash
START_NAV=false EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

发布左转：

```bash
ros2 topic pub -r 2 /ugv_nav_cmd std_msgs/msg/String \
"{data: '{\"mode\":\"TURN_LEFT\",\"raw_left\":-0.3,\"raw_right\":0.3}'}"
```

预期：

- left PWM 低于 `1500`
- right PWM 高于 `1500`
- 左右轮反方向转

停止 publisher：按 `Ctrl+C`。

### Command Timeout 测试

`START_NAV=false` 时，发布一次 command：

```bash
ros2 topic pub --once /ugv_nav_cmd std_msgs/msg/String \
"{data: '{\"mode\":\"FORWARD\",\"raw_left\":0.3,\"raw_right\":0.3}'}"
```

等一秒后检查：

```bash
ros2 topic echo /motor_controller/status --once --full-length
```

预期：

- bridge 回到 neutral PWM，接近 `[1500, 1500]`
- command age 大于 timeout

### Competition Mock Map 测试

```bash
COMPETITION_MODE=true START_CORNER=lower_left START_MOCK_FIELD_MAP=true \
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

预期：

- 不需要手动发布 `/ugv_goal`
- `/ugv_nav_status` 显示 competition phase，比如 `startup_to_center`、`target_nav` 或 `center_loiter`
- motor 输出显示 `DRY RUN`

## 13. 参数速查

`jetson_bringup.sh` 环境变量：

| Variable | Default | Purpose |
| --- | --- | --- |
| `EXTRA_SETUP_BASH` | empty | 额外 workspace setup，通常是 `~/ugv_ws_albert/install/setup.bash` |
| `BENCH_TEST` | `false` | 打开 dry-run、debug status、自动 bench goal |
| `MOTOR_DRY_RUN` | `false` | 不向 controller 写 PWM |
| `START_NAV` | `true` | 是否启动 navigation process |
| `START_MOTOR_CONTROLLER` | `true` | 是否启动 motor bridge |
| `START_DEBUG_STATUS` | `true` | 是否启动综合 debug status node |
| `START_BENCH_GOAL` | `false` | 是否自动发布 `/ugv_goal` |
| `BENCH_GOAL_X_M` | `12.2` | 自动 bench goal x，单位 meter |
| `BENCH_GOAL_Y_M` | `12.0` | 自动 bench goal y，单位 meter |
| `COMPETITION_MODE` | `false` | 打开四角开局和 startup-to-center mission |
| `START_CORNER` | `lower_left` | 四个 competition start corner 之一 |
| `START_MOCK_FIELD_MAP` | `false` | 发布 mock 15x15 field map |
| `MOCK_MARKER_CELL` | `7,7` | Mock marker row,col |
| `LIDAR_PORT` | `/dev/ttyUSB0` | LiDAR serial device |
| `MOTOR_PORT` | `/dev/ttyACM0` | Teensy serial device |
| `FUSION_LIDAR_FRONT_FOV_DEG` | `70.0` | LiDAR front sector 宽度 |
| `INVERT_LEFT_COMMAND` | `false` | 翻转 left motor command |
| `INVERT_RIGHT_COMMAND` | `false` | 翻转 right motor command |
| `INVERT_LEFT_ENCODER` | `false` | 翻转 left encoder sign |
| `INVERT_RIGHT_ENCODER` | `false` | 翻转 right encoder sign |

## 14. 常见问题快速判断

- `/motor_controller/connected` 一直是 `false`: serial port 错、baud 错、或 Teensy firmware 没刷好
- `/encoder_ticks_stamped` 没数据但 connected 是 `true`: encoder wiring 或 firmware serial output 有问题
- `/sensors/synced_summary` 里 `encoder_available: false`: encoder timestamp 太旧
- 发 goal 后 `/ugv_nav_cmd` 没数据: nav 没收到 `/sensors/nav_frame` 或 `/ugv_goal`
- `front_lidar_range_m` 对前方障碍没反应: LiDAR heading alignment 可能需要校正
- `valid_depth_samples` 是 0: ZED depth stream 缺失或 invalid
