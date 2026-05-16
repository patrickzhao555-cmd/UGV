# UGV Jetson/Nano 中文运行手册

English version: [JETSON_BRINGUP_CHECKLIST.md](JETSON_BRINGUP_CHECKLIST.md)

这份文档是当前分支的详细运行手册：
`nav2-inspired-mini-controller`。

默认假设：

- Nano 上 repo 路径：`~/ugv_project`
- ROS workspace：`~/ugv_project/ros2_ws`
- ROS 版本：Humble
- ZED/SLLidar underlay：`~/ugv_ws_albert/install/setup.bash`
- 停止整套 launch：在 launch terminal 按 `Ctrl+C`

## 1. 拉代码和编译

```bash
cd ~/ugv_project
git fetch origin
git checkout nav2-inspired-mini-controller
git pull --ff-only origin nav2-inspired-mini-controller

cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每开一个新的 terminal，只要要跑 ROS 命令，都先 source：

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
```

如果 pull 后运行日志看起来像旧版本，清一次 overlay 再编译：

```bash
cd ~/ugv_project/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. 可选依赖

marker vision 和 dashboard 需要 OpenCV/cv_bridge：

```bash
sudo apt update
sudo apt install -y python3-opencv ros-humble-cv-bridge
```

YOLO 语义障碍辅助是可选的：

```bash
python3 -m pip install --user --force-reinstall "numpy==1.26.4"
python3 -m pip install --user "ultralytics" "numpy<2"
```

ROS Humble 上要保持 NumPy 1.x。如果看到 `_ARRAY_API not found` 或
`numpy.core.multiarray failed to import`，运行：

```bash
python3 -m pip install --user --force-reinstall "numpy==1.26.4"
```

## 3. 主启动命令

主入口：

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

推荐的室内实车运行：

```bash
cd ~/ugv_project/ros2_ws
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 \
START_YOLO_OBSTACLES=true START_DEBUG_DASHBOARD=true \
MOTOR_PORT=/dev/ttyACM0 LIDAR_PORT=/dev/ttyUSB0 MOTOR_DRY_RUN=false \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

更安全的架空或 bench 版本：

```bash
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 \
START_YOLO_OBSTACLES=true START_DEBUG_DASHBOARD=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

如果你想先单独隔离底盘和路径，不跑可选 CV：

```bash
ROUND_MODE=indoor START_YOLO_OBSTACLES=false START_DEBUG_DASHBOARD=false \
START_MARKER_VISION=false MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

当前默认会启动 marker vision。如果只想做最小传感器和导航测试，显式加
`START_MARKER_VISION=false`。

## 4. TigerVNC 和 Dashboard

Nano 上安装 TigerVNC/XFCE：

```bash
sudo apt update
sudo apt install -y tigervnc-standalone-server tigervnc-common xfce4 xfce4-goodies dbus-x11 x11-utils
vncpasswd
mkdir -p ~/.vnc
cat > ~/.vnc/xstartup <<'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
startxfce4 &
EOF
chmod +x ~/.vnc/xstartup
bash ~/ugv_project/ros2_ws/tools/start_vnc.sh
```

Windows 上用 TigerVNC Viewer 连接：

```text
129.65.74.107:1
# 或者
129.65.74.107:5901
```

如果想让 Nano 启动/登录后自动打开 VNC：

```bash
bash ~/ugv_project/ros2_ws/tools/install_vnc_autostart.sh
systemctl --user status tigervnc-ugv.service
```

如果你不是从 VNC 桌面 terminal 启动，而是从 SSH 启动：

```bash
export DISPLAY=:1
```

Dashboard 启动：

```bash
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 START_DEBUG_DASHBOARD=true \
START_YOLO_OBSTACLES=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Dashboard 快捷键：

- `s`：截图保存到 `~/ugv_dashboard_screenshots`
- `q` 或 `Esc`：关闭窗口

常用 dashboard 参数：

```bash
DASHBOARD_UPDATE_HZ=4.0
DASHBOARD_CAMERA_SEARCH_DEPTH_M=0.30
```

## 5. 直接电机测试

这是调电机方向和 encoder 符号最快的方式。第一次必须架空 UGV。不要和完整
navigation bringup 同时运行，因为两者都会写 `/ugv_nav_cmd`。

```bash
cd ~/ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
source install/setup.bash

ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=forward raw:=0.22 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=false invert_right_command:=true
```

可选 motion：

```text
forward
backward
turn_left
turn_right
left_forward
left_backward
right_forward
right_backward
raw
sequence
```

raw 例子：

```bash
ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=raw raw_left:=0.20 raw_right:=0.20 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=false invert_right_command:=true
```

检查标准：

- `motion:=forward` 必须让车朝前方传感器方向移动。
- 前进时左右 encoder tick 应该同号。
- 原地转向时左右 encoder tick 应该反号。
- 这个测试是 open-loop；`duration_s:=0.8 raw:=0.22` 主要用来确认方向。真实转角请看输出里的 `est_yaw`。
- 如果物理前进变成后退，先同时翻转左右 command inversion。
- 如果物理前进时一侧前转、一侧后转，先翻转物理方向错误那一侧的 command inversion。
- 只有在轮子物理方向已经正确、但 odometry 符号仍然错误时，才翻转 encoder inversion。

当前底盘在 `jetson_bringup.sh` 里的默认值：

```bash
INVERT_LEFT_COMMAND=false
INVERT_RIGHT_COMMAND=true
INVERT_LEFT_ENCODER=false
INVERT_RIGHT_ENCODER=false
```

## 6. Bench 和 Dry-Run

推荐 bench mode：

```bash
cd ~/ugv_project/ros2_ws
BENCH_TEST=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

手动 dry-run 目标点测试：

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

15 yard x 15 yard 场地内部是 `13.716 m x 13.716 m`，中心大约是
`(6.86, 6.86)`。

## 7. Round 和比赛模式

Round 1：

```bash
ROUND_MODE=round1 EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Round 2：

```bash
ROUND_MODE=round2 EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Round 3 / competition：

```bash
ROUND_MODE=round3 UGV_START_X_M=0.46 UGV_START_Y_M=0.46 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Mock target dry-run：

```bash
COMPETITION_MODE=true UGV_START_X_M=0.46 UGV_START_Y_M=0.46 \
START_MOCK_FIELD_MAP=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

推荐 ESP/UAV target topic：

```bash
ros2 topic pub --once /ugv/target std_msgs/msg/String \
"{data: '{\"type\":\"ugv_target_v1\",\"source\":\"uav\",\"unit\":\"m\",\"frame\":\"field_bottom_left\",\"x\":6.86,\"y\":6.86}'}"
```

旧的 `/ugv/field_map` JSON matrix 仍然兼容，但正常情况下 UAV/ESP 只需要发目标坐标。
障碍物由 UGV 自己用 LiDAR/ZED/YOLO 实时感知。

## 8. Marker Vision

训练图片放这里：

```text
ros2_ws/src/ugv_perception/training/marker_images/
```

训练 ORB model：

```bash
cd ~/ugv_project/ros2_ws
python3 src/ugv_perception/ugv_perception/marker_model_trainer.py \
  --image-dir src/ugv_perception/training/marker_images \
  --model-out src/ugv_perception/models/marker_orb_model.npz \
  --max-descriptors 65000
```

单独 live CV 测试，不启动 motor/nav/LiDAR：

```bash
cd ~/ugv_project/ros2_ws
MARKER_VISION_TEST=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

常用 topic：

```bash
ros2 topic echo /ugv/marker_vision_debug --full-length
ros2 topic echo /ugv/marker_detection --once
ros2 topic echo /ugv/uav_flag --once --full-length
```

## 9. YOLO 语义障碍辅助

YOLO 只做辅助。它把 chair、table、person 等语义物体变成额外 obstacle points，
让规划更保守。YOLO 漏检不会移除 LiDAR/ZED 的安全障碍。

启用：

```bash
START_YOLO_OBSTACLES=true YOLO_MODEL_PATH=yolov8n.pt YOLO_DEVICE=auto \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

如果 CUDA/PyTorch 不稳定：

```bash
YOLO_DEVICE=cpu
```

默认识别类别：

```text
person,chair,couch,dining table,bench,potted plant,backpack,suitcase
```

Debug topics：

```bash
ros2 topic echo /sensors/yolo_semantic_debug --full-length
ros2 topic echo /sensors/yolo_semantic_obstacle_points --once
```

## 10. 传感器和导航设计

几个关键问题的答案：

- 车头朝向：默认通过左右 encoder odometry 估计。ZED IMU yaw-rate 可以融合，但在轴向和正负号校准前默认关闭。
- 车身尺寸：导航用 `ROBOT_LENGTH_M`、`ROBOT_WIDTH_M`、`ROBOT_TRACK_WIDTH_M`、障碍膨胀和安全 margin 建模 30 inch x 30 inch 底盘。
- LiDAR 视野：室内默认 `LIDAR_USED_FOV_DEG=180.0`，并且 `NAV_ALLOW_REVERSE=false`。
- ZED depth：fusion 里有 ground-aware depth filter，会尽量忽略地面，把锥桶、箱子、桶、桌椅腿这种竖直物体变成障碍点。
- YOLO：只做语义膨胀，不负责硬安全。
- Map：当前 live map 主要是 obstacle-focused map，indoor search 还有粗 visited cells。Dashboard 会显示 session-local camera-searched wedge map，用 110 度 FOV 和 `DASHBOARD_CAMERA_SEARCH_DEPTH_M=0.30`。但 camera coverage 还不是 planner 的硬目标。

运行层级：

```text
ZED + LiDAR + encoder adapters
        -> fusion_node (/sensors/nav_frame)
        -> ugv_nav_dual_mode.py
        -> /ugv_nav_cmd
        -> motor_controller_bridge
        -> Teensy
```

## 11. 连续局部控制器

当前分支保留全局 search/marker 逻辑，但默认使用 Nav2-inspired 连续局部控制：

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=true
```

它会采样 forward `(v_mps, omega_radps)`，前向模拟 0.4 到 2.0 秒轨迹，根据目标进度、
障碍距离、polar gap 宽度、朝向、速度和平滑度打分，然后在速度层做加速度限制和低通滤波。

Collision Monitor 风格安全层仍然会在近距离障碍前限速或停下，但普通路径选择不再依赖旧的六扇区最小距离动作块。

常用参数：

```bash
NAV_CONTINUOUS_MAX_SPEED_MPS=0.36
NAV_CONTINUOUS_MAX_OMEGA_RPS=1.15
NAV_CONTINUOUS_HORIZON_S=1.35
NAV_CONTINUOUS_ACCEL_LIMIT_MPS2=0.35
NAV_CONTINUOUS_OMEGA_ACCEL_LIMIT_RPS2=1.80
NAV_CONTINUOUS_LOWPASS_ALPHA=0.55
NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M=1.20
NAV_CONTINUOUS_STOP_CLEARANCE_M=0.48
NAV_CONTINUOUS_GAP_BUFFER_M=0.025
NAV_CONTINUOUS_LATENCY_BUFFER_S=0.25
```

如果室内测试感觉太激进：

```bash
NAV_CONTINUOUS_MAX_SPEED_MPS=0.28
NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M=1.30
```

对比旧局部规划：

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=false
```

## 12. ZED Depth 障碍过滤和主动扫视

Depth ground filter：

```bash
FUSION_DEPTH_GROUND_FILTER_ENABLED=true
FUSION_DEPTH_PROJECTION_STRIDE_PX=8
FUSION_DEPTH_GROUND_MIN_DELTA_M=0.18
FUSION_DEPTH_GROUND_RATIO=0.88
FUSION_DEPTH_OBSTACLE_MIN_COMPONENT_HEIGHT_PX=14
FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M=0.42
```

主动扫视恢复：

```bash
NAV_ACTIVE_SCAN_ENABLED=true
NAV_ACTIVE_SCAN_CONFIRM_STEPS=3
NAV_ACTIVE_SCAN_STEPS=7
NAV_ACTIVE_SCAN_COOLDOWN_STEPS=4
NAV_ACTIVE_SCAN_PROBE_STEPS=5
NAV_ACTIVE_SCAN_FRONT_CLEAR_M=1.25
NAV_ACTIVE_SCAN_CORRIDOR_EXTRA_WIDTH_M=0.03
```

每次 active scan 结束后，程序会短暂进入 probe/cooldown 状态，先让连续控制器尝试刚刚扫到的视野；如果仍然无法形成安全弧线，下一次扫描会强制换到另一侧，避免只看左边或只看右边就卡死。

错位的椅子/桌腿通道现在由连续控制器的弧线 rollout 判断，而不是把障碍当成正
面平行墙来量一个死板宽度。局部安全层默认使用真实车身 footprint 加
`ROBOT_OBSTACLE_BUFFER_M=0.025`，不再额外叠加很大的 local inflation。若要保守
测试，可以调高 `ROBOT_OBSTACLE_BUFFER_M` 或 `NAV_LOCAL_PLAN_INFLATION_M`，但超过
约 `0.05` 后，30 inch 车身会很容易把 34-35 inch 的通道判断成不可通过。
目的：如果前方连续几帧确实受阻，或者连续控制器找不到安全轨迹，UGV 会原地朝更空的一侧转动，
扩大 LiDAR/ZED 视野，而不是卡在原地不动。

现在 active scan 会更重视 ZED depth 的前方走廊障碍；如果连续控制器已经降到接近原地犹豫，而最佳 gap 在侧方，UGV 会更早朝更开阔的一侧原地扫描，不再把 75-90 度侧向空隙当成“正前方 clear”。

注意：1.25m 内看到一个障碍不等于“死路”。程序会用当前 LiDAR/ZED 点云评估车身 footprint 能否沿前方弧线路径穿过障碍间 gap；只有没有可通过弧线，或者控制器已经接近原地犹豫时，才触发主动扫视。

## 13. Debug Topics

```bash
ros2 topic echo /ugv/debug_status --full-length
ros2 topic echo /ugv_nav_status --full-length
ros2 topic echo /sensors/synced_summary --once --full-length
ros2 topic echo /motor_controller/status --once --full-length
ros2 topic echo /encoder_ticks_stamped --once
ros2 topic echo /zed/status --once --full-length
```

重点看这些字段：

- `front_lidar`、`depth_roi`、`front_clearance`、`clear_src`
- `semantic_pts`
- `phase`、`cmd`、`pose`
- `odom_warn`
- `velocity_control`
- `local_costmap`
- `active_scan`
- `zed_obstacle_points`、`depth_obstacle_points`、`depth_obstacle_points_filtered`

## 14. 常见故障快速判断

- `_ARRAY_API not found`：NumPy 2.x 破坏了 ROS Humble Python binary，重装 `numpy==1.26.4`。
- `FORWARD` 物理上后退：command inversion 错了，先修这个再调导航。
- `forward_command_has_opposite_encoder_signs`：encoder 符号校准错了。
- `/ugv_nav_cmd` 没数据：nav 没收到 `/sensors/nav_frame`，或者没有目标/search mode。
- `front_lidar_range_m` 对前方障碍没反应：LiDAR 0 度方向没有对准车头。
- `valid_depth_samples` 为 0：ZED depth stream 缺失或 invalid。
- Dashboard 不打开：从 VNC terminal 启动，或者设置 `DISPLAY=:1`。
- YOLO 没识别桌腿：这是预期限制，主要安全仍然靠 ZED depth obstacle points 和 LiDAR。

## 15. 开发注意事项

- 不要提交 `ros2_ws/build`、`ros2_ws/install`、`ros2_ws/log`。
- package-level README 保持短小；启动行为变化时，优先更新这份中文手册和英文手册。
- commit 前运行 `git diff --check`。
- Windows 本地适合做 Python syntax check；完整 `colcon build` 要在 Nano/ROS 环境跑。

## 16. Velocity PID / Replay Metrics

导航现在会在 `/ugv_nav_cmd` 中明确发布速度意图，同时保留 raw 兼容字段：

```json
{"command_type":"velocity","v_mps":0.18,"omega_radps":0.35,"raw_left":0.0,"raw_right":0.0}
```

电机桥默认仍然使用 raw 模式，速度闭环默认关闭：

```bash
MOTOR_VELOCITY_CONTROL_ENABLED=false
NAV_EMIT_VELOCITY_COMMANDS=true
```

第一次打开速度 PID 前必须架空车体：

```bash
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=1 MOTOR_DRY_RUN=false \
MOTOR_VELOCITY_CONTROL_ENABLED=true \
MOTOR_VELOCITY_KP=0.80 MOTOR_VELOCITY_KI=0.0 MOTOR_VELOCITY_KD=0.02 \
MOTOR_VELOCITY_FEEDFORWARD_RAW_PER_MPS=1.35 \
MOTOR_VELOCITY_MAX_TARGET_MPS=0.35 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

检查状态：

```bash
ros2 topic echo /motor_controller/status
```

重点看 `control_mode`、`target_left_mps`、`target_right_mps`、
`measured_left_mps`、`measured_right_mps`、`pid_left`、`pid_right`、
`velocity_safe_reason`。如果 encoder 速度缺失或过期，默认会输出 neutral
PWM；第一次测试不要打开 `MOTOR_VELOCITY_FALLBACK_TO_RAW_WITHOUT_ENCODER`。

离线 replay/status 指标：

```bash
python3 ros2_ws/src/ugv_nav/tools/replay_nav_metrics.py path/to/replay_or_status.jsonl
python3 ros2_ws/src/ugv_nav/tools/replay_nav_metrics.py --json-only path/to/replay_or_status.jsonl
```
