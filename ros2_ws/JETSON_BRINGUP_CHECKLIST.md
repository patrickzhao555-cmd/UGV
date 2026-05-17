# UGV Jetson/Nano Runbook

Chinese version: [JETSON_BRINGUP_CHECKLIST_ZH.md](JETSON_BRINGUP_CHECKLIST_ZH.md)

This is the detailed operating guide for the current branch:
`nav2-inspired-mini-controller`.

Assumptions:

- Repo path on the Nano: `~/ugv_project`
- ROS workspace: `~/ugv_project/ros2_ws`
- ROS distro: Humble
- Extra ZED/SLLidar underlay when needed: `~/ugv_ws_albert/install/setup.bash`
- Stop any launch with `Ctrl+C`

## 1. Pull and Build

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

Every new terminal that runs ROS commands needs the same overlays:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash
```

If runtime behavior looks stale after a pull, rebuild from a clean overlay once:

```bash
cd ~/ugv_project/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Optional Dependencies

Marker vision and dashboard need OpenCV/cv_bridge:

```bash
sudo apt update
sudo apt install -y python3-opencv ros-humble-cv-bridge
```

YOLO semantic obstacle assist is optional:

```bash
python3 -m pip install --user --force-reinstall "numpy==1.26.4"
python3 -m pip install --user "ultralytics" "numpy<2"
```

Keep ROS Humble on NumPy 1.x. If you see `_ARRAY_API not found` or
`numpy.core.multiarray failed to import`, repair the local Python environment:

```bash
python3 -m pip install --user --force-reinstall "numpy==1.26.4"
```

## 3. Main Launcher

The main entry point is:

```bash
cd ~/ugv_project/ros2_ws
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Recommended indoor ground run:

```bash
cd ~/ugv_project/ros2_ws
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 \
START_YOLO_OBSTACLES=true START_DEBUG_DASHBOARD=true \
MOTOR_PORT=/dev/ttyACM0 LIDAR_PORT=/dev/ttyUSB0 MOTOR_DRY_RUN=false \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Safer lifted/bench variant:

```bash
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 \
START_YOLO_OBSTACLES=true START_DEBUG_DASHBOARD=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Disable optional perception pieces if you need to isolate drivetrain/pathing:

```bash
ROUND_MODE=indoor START_YOLO_OBSTACLES=false START_DEBUG_DASHBOARD=false \
START_MARKER_VISION=false MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

The current default starts marker vision. Use `START_MARKER_VISION=false` when
you want a minimal sensor/nav test.

## 4. TigerVNC and Dashboard

Install TigerVNC/XFCE on the Nano:

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

Connect from Windows with TigerVNC Viewer to:

```text
129.65.74.107:1
# or
129.65.74.107:5901
```

To start VNC automatically after the Nano boots/logs in:

```bash
bash ~/ugv_project/ros2_ws/tools/install_vnc_autostart.sh
systemctl --user status tigervnc-ugv.service
```

If you launch from SSH instead of a VNC terminal:

```bash
export DISPLAY=:1
```

Dashboard launch:

```bash
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=2 START_DEBUG_DASHBOARD=true \
START_YOLO_OBSTACLES=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Dashboard keys:

- `s`: save screenshot to `~/ugv_dashboard_screenshots`
- `q` or `Esc`: close dashboard

Useful dashboard overrides:

```bash
DASHBOARD_UPDATE_HZ=4.0
DASHBOARD_CAMERA_SEARCH_DEPTH_M=0.30
```

## 5. Direct Drivetrain Tests

Use this before tuning navigation. Lift the UGV for the first run. Do not run
the full navigation bringup at the same time.

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

Available motions:

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

Raw example:

```bash
ros2 launch ugv_motor_controller motor_direct_test.launch.py \
  motion:=raw raw_left:=0.20 raw_right:=0.20 duration_s:=0.8 yes:=true \
  port:=/dev/ttyACM0 dry_run:=false \
  invert_left_command:=false invert_right_command:=true
```

Expected checks:

- `motion:=forward` physically moves the chassis toward the front sensors.
- Forward left/right encoder ticks should have the same sign.
- Turning left/right encoder ticks should have opposite signs.
- The test is open-loop; `duration_s:=0.8 raw:=0.22` is mainly for direction
  checking. Use the printed `est_yaw` to measure actual turn angle.
- If physical forward goes backward, flip both command inversion flags together.
- If physical forward makes one side drive backward and the other side drive forward, flip the command inversion on the wrong physical side.
- Only flip encoder inversion after the wheels physically move in the right directions but odometry signs are still wrong.
- Ground odometry calibration for the current chassis measured raw `0.14` for
  `1.5s`: average encoder ticks `1027`, old odom estimate `0.3872m`, actual
  travel `0.1800m`, scale `0.4649`. Test nav odometry with
  `ROBOT_TICKS_PER_REV=2151`.

Current chassis defaults in `jetson_bringup.sh`:

```bash
INVERT_LEFT_COMMAND=false
INVERT_RIGHT_COMMAND=true
INVERT_LEFT_ENCODER=false
INVERT_RIGHT_ENCODER=false
ROBOT_WHEEL_RADIUS_M=0.06
ROBOT_TICKS_PER_REV=1000
```

For the current calibrated chassis, override the default when running nav:

```bash
ROBOT_TICKS_PER_REV=2151
```

## 6. Velocity PID Bench Test

Navigation now publishes velocity intent on `/ugv_nav_cmd`:

```json
{"command_type":"velocity","v_mps":0.18,"omega_radps":0.35,"raw_left":0.0,"raw_right":0.0}
```

The bridge still defaults to raw mode for safety:

```bash
MOTOR_VELOCITY_CONTROL_ENABLED=false
NAV_EMIT_VELOCITY_COMMANDS=true
```

Lift the UGV before enabling closed-loop wheel-speed control:

```bash
ROUND_MODE=indoor DRIVE_SPEED_LEVEL=1 MOTOR_DRY_RUN=false \
MOTOR_VELOCITY_CONTROL_ENABLED=true \
MOTOR_VELOCITY_KP=0.80 MOTOR_VELOCITY_KI=0.0 MOTOR_VELOCITY_KD=0.02 \
MOTOR_VELOCITY_FEEDFORWARD_RAW_PER_MPS=1.35 \
MOTOR_VELOCITY_MAX_TARGET_MPS=0.35 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Watch the bridge status:

```bash
ros2 topic echo /motor_controller/status
```

Key fields:

- `control_mode`: `raw`, `velocity_pid`, `velocity_safe_neutral`, or fallback
- `target_left_mps`, `target_right_mps`
- `measured_left_mps`, `measured_right_mps`
- `velocity_error_left_mps`, `velocity_error_right_mps`
- `pid_left`, `pid_right`
- `velocity_safe_reason`
- `encoder_speed_dt_source`: `controller_millis`, `host_time`, or unavailable
- `encoder_speed_anomaly`: empty when normal, otherwise describes clamping/fallback

Default stale-encoder behavior is neutral PWM. Do not set
`MOTOR_VELOCITY_FALLBACK_TO_RAW_WITHOUT_ENCODER=true` during first hardware
tests.

Encoder speed estimation defaults prefer Teensy `controller_millis` when present
and fall back to host time only when needed:

```bash
MOTOR_VELOCITY_ENCODER_SPEED_FILTER_ALPHA=0.65
MOTOR_VELOCITY_ENCODER_SPEED_MAX_MPS=2.0
```

## 7. Bench and Dry-Run Modes

Recommended bench mode:

```bash
cd ~/ugv_project/ros2_ws
BENCH_TEST=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Manual dry-run with a goal:

Terminal 1:

```bash
cd ~/ugv_project/ros2_ws
MOTOR_DRY_RUN=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws_albert/install/setup.bash
cd ~/ugv_project/ros2_ws
source install/setup.bash

ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 12.2, y: 12.0, z: 0.0}}"
```

The 15 yard x 15 yard field is `13.716 m x 13.716 m`; center is about
`(6.86, 6.86)`.

## 8. Round and Competition Modes

Round 1:

```bash
ROUND_MODE=round1 EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Round 2:

```bash
ROUND_MODE=round2 EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Round 3 / competition:

```bash
ROUND_MODE=round3 UGV_START_X_M=0.46 UGV_START_Y_M=0.46 \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Mock target dry-run:

```bash
COMPETITION_MODE=true UGV_START_X_M=0.46 UGV_START_Y_M=0.46 \
START_MOCK_FIELD_MAP=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Preferred ESP/UAV target topic:

```bash
ros2 topic pub --once /ugv/target std_msgs/msg/String \
"{data: '{\"type\":\"ugv_target_v1\",\"source\":\"uav\",\"unit\":\"m\",\"frame\":\"field_bottom_left\",\"x\":6.86,\"y\":6.86}'}"
```

The legacy `/ugv/field_map` JSON matrix is still accepted for old bench tests,
but the UAV/ESP side should normally only send a target coordinate. Obstacles
come from live LiDAR/ZED/YOLO perception.

## 9. Marker Vision

Training images go here:

```text
ros2_ws/src/ugv_perception/training/marker_images/
```

Train the lightweight ORB model:

```bash
cd ~/ugv_project/ros2_ws
python3 src/ugv_perception/ugv_perception/marker_model_trainer.py \
  --image-dir src/ugv_perception/training/marker_images \
  --model-out src/ugv_perception/models/marker_orb_model.npz \
  --max-descriptors 65000
```

Standalone live CV test, with no motor/nav/LiDAR stack:

```bash
cd ~/ugv_project/ros2_ws
MARKER_VISION_TEST=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Useful topics:

```bash
ros2 topic echo /ugv/marker_vision_debug --full-length
ros2 topic echo /ugv/marker_detection --once
ros2 topic echo /ugv/uav_flag --once --full-length
```

## 10. YOLO Semantic Obstacle Assist

YOLO is advisory. It adds chair/table/person-style points for conservative
inflation; missed YOLO detections do not remove LiDAR/ZED safety obstacles.

Enable:

```bash
START_YOLO_OBSTACLES=true YOLO_MODEL_PATH=yolov8n.pt YOLO_DEVICE=auto \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

If CUDA/PyTorch is unstable:

```bash
YOLO_DEVICE=cpu
```

Default classes:

```text
person,chair,couch,dining table,bench,potted plant,backpack,suitcase
```

Debug topics:

```bash
ros2 topic echo /sensors/yolo_semantic_debug --full-length
ros2 topic echo /sensors/yolo_semantic_obstacle_points --once
```

## 11. Sensor and Navigation Design

Answers to common design questions:

- Heading: the robot estimates heading from left/right encoder odometry by default. ZED IMU yaw-rate blending exists but remains off until axis/sign calibration is verified.
- Footprint: navigation models the 30 inch by 30 inch chassis using `ROBOT_LENGTH_M`, `ROBOT_WIDTH_M`, `ROBOT_TRACK_WIDTH_M`, obstacle inflation, and front/rear margins.
- Ground odometry scale: navigation uses `ROBOT_WHEEL_RADIUS_M` and
  `ROBOT_TICKS_PER_REV`. The current chassis calibration recommends
  `ROBOT_TICKS_PER_REV=2151`; if velocity PID is later used for true ground
  speed, calibrate `MOTOR_TICKS_PER_REV` consistently too.
- LiDAR field of view: indoor default uses `LIDAR_USED_FOV_DEG=180.0` and `NAV_ALLOW_REVERSE=false`.
- Camera/ZED: ZED depth is ground-aware filtered before fusion so floor pixels do not become obstacles.
- YOLO: optional semantic inflation only; LiDAR/ZED depth remain collision safety.
- Map: the live map is obstacle-focused and search keeps coarse visited cells. The dashboard visualizes a session-local camera-searched wedge map using 110 degree FOV and `DASHBOARD_CAMERA_SEARCH_DEPTH_M=0.30`. Camera coverage is not yet a hard planner objective.

Runtime layer flow:

```text
ZED + LiDAR + encoder adapters
        -> fusion_node (/sensors/nav_frame)
        -> ugv_nav_dual_mode.py
        -> /ugv_nav_cmd
        -> motor_controller_bridge
        -> Teensy
```

## 12. Continuous Local Controller

The current branch keeps global search/marker logic but uses a Nav2-inspired
continuous local controller by default:

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=true
```

It samples forward `(v_mps, omega_radps)`, simulates 0.4-2.0 s trajectories,
scores target progress, obstacle clearance, polar gap width, heading, speed, and
smoothness, then applies acceleration limits and low-pass filtering at the
velocity layer.

Collision Monitor style safety still caps or stops forward speed near obstacles,
but ordinary path choice is done by sampled trajectories rather than the old
six-sector minimum-distance action blocks.

Useful overrides:

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
NAV_EMIT_VELOCITY_COMMANDS=true
NAV_LOCAL_COSTMAP_ENABLED=true
NAV_CONTINUOUS_ALLOW_COSTMAP_SOFT_PENALTY=false
NAV_LOCAL_COSTMAP_WIDTH_M=4.0
NAV_LOCAL_COSTMAP_HEIGHT_M=4.0
NAV_LOCAL_COSTMAP_RESOLUTION_M=0.06
NAV_LOCAL_COSTMAP_DYNAMIC_DECAY_S=1.0
NAV_LOCAL_COSTMAP_OBSTACLE_RADIUS_M=0.06
NAV_LOCAL_COSTMAP_INFLATION_M=0.08
NAV_LOCAL_COSTMAP_LIDAR_CLEAR_RADIUS_M=0.05
NAV_LOCAL_COSTMAP_MAX_RAYTRACE_M=4.0
```

The rolling local costmap ray-clears both LiDAR hit beams and no-hit/max-range
beams. Ordinary LiDAR/ZED/YOLO observations stay in this local dynamic layer
instead of being written permanently into the global `known_costmap`; field-map
obstacles and deliberate blocked recovery patches remain persistent.

If the robot feels too aggressive indoors:

```bash
NAV_CONTINUOUS_MAX_SPEED_MPS=0.28
NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M=1.30
```

Compare against the older local planner:

```bash
NAV_CONTINUOUS_CONTROL_ENABLED=false
```

## 13. ZED Depth Obstacle Filtering and Active Scan

Ground-aware depth filter:

```bash
FUSION_DEPTH_GROUND_FILTER_ENABLED=true
FUSION_DEPTH_PROJECTION_STRIDE_PX=8
FUSION_DEPTH_GROUND_MIN_DELTA_M=0.18
FUSION_DEPTH_GROUND_RATIO=0.88
FUSION_DEPTH_OBSTACLE_MIN_COMPONENT_HEIGHT_PX=14
FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M=0.42
```

Active scan recovery:

```bash
NAV_ACTIVE_SCAN_ENABLED=true
NAV_ACTIVE_SCAN_CONFIRM_STEPS=3
NAV_ACTIVE_SCAN_STEPS=7
NAV_ACTIVE_SCAN_COOLDOWN_STEPS=4
NAV_ACTIVE_SCAN_PROBE_STEPS=5
NAV_ACTIVE_SCAN_FRONT_CLEAR_M=1.25
NAV_ACTIVE_SCAN_CORRIDOR_EXTRA_WIDTH_M=0.03
```

Tight, staggered chair/table gaps are handled by the continuous controller's
rolled-out arc checks. The local safety layer now relies on the real robot
footprint plus `ROBOT_OBSTACLE_BUFFER_M=0.025` instead of double-counting a
large local inflation. If you need an extra cautious run, increase
`ROBOT_OBSTACLE_BUFFER_M` or `NAV_LOCAL_PLAN_INFLATION_M`, but values above
about `0.05` can make 34-35 inch gaps look physically impossible for the
30 inch chassis.

Purpose: ZED/LiDAR points in front are evaluated as a footprint-sized corridor
gap, not as a single nearest obstacle. If there is still a forward arc through
the point cloud, the controller keeps driving/steering through the gap. If the
front corridor has no passable arc, or the continuous controller is reduced to
near-zero progress while the best opening is lateral, the UGV turns in place
toward the clearer side to build a wider LiDAR/ZED view. A side gap at 75-90
degrees is no longer allowed to raise the straight-ahead speed cap by itself.
After one active-scan burst, the controller briefly probes the newly observed
view and blocks an immediate same-side re-scan; if it still cannot make a safe
arc, the next scan is forced to the opposite side.

## 14. Debug Topics

```bash
ros2 topic echo /ugv/debug_status --full-length
ros2 topic echo /ugv_nav_status --full-length
ros2 topic echo /sensors/synced_summary --once --full-length
ros2 topic echo /motor_controller/status --once --full-length
ros2 topic echo /encoder_ticks_stamped --once
ros2 topic echo /zed/status --once --full-length
```

Important fields:

- `front_lidar`, `depth_roi`, `front_clearance`, `clear_src`
- `semantic_pts`
- `phase`, `cmd`, `pose`
- `odom_warn`
- `velocity_control`
- `local_costmap`
- `active_scan`
- `zed_obstacle_points`, `depth_obstacle_points`, `depth_obstacle_points_filtered`

Offline replay/status metrics:

```bash
python3 ros2_ws/src/ugv_nav/tools/replay_nav_metrics.py path/to/replay_or_status.jsonl
python3 ros2_ws/src/ugv_nav/tools/replay_nav_metrics.py --json-only path/to/replay_or_status.jsonl
```

The tool accepts raw JsonReplayBridge frames and mixed `REAL CMD`/`NAV STATUS`
logs. Missing fields are reported in the JSON summary instead of treated as a
failure.

## 15. Fast Failure Clues

- `_ARRAY_API not found`: NumPy 2.x broke ROS Humble Python binaries. Reinstall `numpy==1.26.4`.
- Robot drives backward on `FORWARD`: command inversion is wrong; fix both sides before tuning.
- `forward_command_has_opposite_encoder_signs`: encoder sign calibration is wrong.
- `/ugv_nav_cmd` is quiet: nav is not receiving `/sensors/nav_frame` or no goal/search mode is active.
- `front_lidar_range_m` does not react to front objects: LiDAR physical zero direction is not aligned with robot front.
- `valid_depth_samples` is zero: ZED depth stream is missing/invalid.
- Dashboard does not open: launch from VNC terminal or set `DISPLAY=:1`.
- YOLO boxes missing on table legs: expected limitation; ZED depth obstacle points and LiDAR remain the primary safety sources.

## 16. Development Notes

- Do not commit `ros2_ws/build`, `ros2_ws/install`, or `ros2_ws/log`.
- Keep package READMEs short; update this runbook and the Chinese runbook when launch behavior changes.
- Use `git diff --check` before committing.
- On Windows/local dev, Python syntax checks are useful; full `colcon build` must be run on the Nano/ROS environment.
