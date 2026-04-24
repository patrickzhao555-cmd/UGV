# Jetson Bring-Up Checklist

## 1. Build the packages you need

```bash
cd ~/UGV/ros2_ws
colcon build --symlink-install --packages-select ugv_sensor_sync ugv_motor_controller
```

## 2. Source the workspace

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

## 3. Start the whole UGV stack from one command

```bash
cd ~/UGV/ros2_ws
bash jetson_bringup.sh
```

If your ports are different, override them without editing code:

```bash
cd ~/UGV/ros2_ws
LIDAR_PORT=/dev/ttyUSB0 MOTOR_PORT=/dev/ttyTHS1 bash jetson_bringup.sh
```

If one side drives backward when it should go forward, flip it from the shell:

```bash
cd ~/UGV/ros2_ws
INVERT_LEFT_COMMAND=true INVERT_LEFT_ENCODER=true bash jetson_bringup.sh
```

## 4. Make sure the Teensy side is ready

- flash `ros2_ws/src/ugv_motor_controller/firmware/teensy_4_1_motor_bridge.ino` to the Teensy 4.1
- confirm Jetson UART is wired to Teensy `Serial1`
- confirm USB is optional debug only

## 5. Validate the raw sensor and drivetrain chain

In new terminals after sourcing the workspace:

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /zed/imu --once
```

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /scan/synced --once
```

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /motor_controller/connected --once
```

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /encoder_ticks_stamped --once
```

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /motor_controller/status --once
```

## 6. Validate the fused middle-layer output

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /sensors/synced_summary --once
```

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /sensors/nav_frame --once
```

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /sensors/front_clearance_m --once
```

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /sensors/near_obstacle --once
```

## 7. Validate navigation and motor-controller connection

Publish one temporary test goal:

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: 3.0, y: 3.0, z: 0.0}}"
```

Then confirm nav starts publishing commands:

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /ugv_nav_cmd --once
```

Then confirm the bridge sees and converts that command:

```bash
cd ~/UGV/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 topic echo /motor_controller/status --once
```

## 8. What good looks like

- `/zed/imu` prints IMU data with a recent timestamp
- `/scan/synced` prints a LaserScan with a recent timestamp
- `/encoder_ticks_stamped` prints recent left/right ticks plus raw wheel ticks
- `/sensors/nav_frame` prints a pathing-ready contract from `sensor_sync`
- `/sensors/synced_summary` shows encoder availability, encoder age, front clearance, and obstacle counts
- `/ugv_nav_cmd` prints a JSON command after a goal is published
- `/motor_controller/status` shows non-neutral PWM values when nav is asking the robot to move

## 9. Fast failure clues

- `/motor_controller/connected` stays `false`: wrong Jetson serial port, wrong baud, or Teensy firmware not flashed
- `/encoder_ticks_stamped` is empty but connected is `true`: encoder wiring or Teensy serial output issue
- `/sensors/synced_summary` shows `encoder_available: false`: encoder timestamps are stale relative to fused sensor frames
- `/ugv_nav_cmd` stays quiet after a goal: nav is not receiving `/sensors/nav_frame` or `/ugv_goal`
