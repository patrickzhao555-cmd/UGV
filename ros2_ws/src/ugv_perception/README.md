# ugv_perception

This package is now centered on one main job:
read ZED 2i depth data and publish a warning flag when something is too close.

## Main node

`obstacle_warning`

- Subscribes to: `/zed/zed_node/depth/depth_registered`
- Publishes: `/ugv/obstacle_warning` as `std_msgs/Bool`
- Publishes: `/ugv/obstacle_distance_m` as `std_msgs/Float32`

It inspects a center-lower region of the depth image, filters invalid pixels,
uses a near-percentile distance estimate, smooths it over a small window,
and applies hysteresis so the warning signal does not flicker.

Run it with:

```bash
cd ~/UGV/ros2_ws
colcon build --symlink-install --packages-select ugv_perception
source install/setup.bash
ros2 run ugv_perception obstacle_warning
```

Useful parameter overrides:

```bash
ros2 run ugv_perception obstacle_warning --ros-args \
  -p threshold_on_m:=0.30 \
  -p threshold_off_m:=0.35 \
  -p roi_w_frac:=0.50 \
  -p roi_h_frac:=0.40 \
  -p roi_y_center_frac:=0.55
```

## Optional debug node

`zed_obj_distance`

- Subscribes to: `/zed/zed_node/obj_det/objects`
- Prints the closest detected ZED object to the console

Run it with:

```bash
ros2 run ugv_perception zed_obj_distance
```

## Kept vs removed

Kept:

- `ugv_perception/obstacle_warning.py`: main depth-warning node
- `ugv_perception/zed_obj_distance.py`: optional debug helper

Removed from the main package flow:

- old direct ZED SDK scripts for accelerometer, gyroscope, magnetometer
- `zed_utils.py`, which only supported those one-off scripts
