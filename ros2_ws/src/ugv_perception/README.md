# ugv_perception

This package is now centered on one main job:
read ZED 2i depth data and publish a warning flag when something is too close.
It also contains the marker-vision training scaffold used by competition mode.

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

## Marker Vision Baseline

This is a hybrid marker detector. It first looks for a generic high-contrast
dark/light marker shape using region, edge, and color-neutrality cues, then uses
the trained ORB model as an additional appearance check/fallback. That matters
because the exact black/white pattern can change while the marker size and style
stay the same.

Dependency on the Jetson/Nano:

```bash
sudo apt install python3-opencv ros-humble-cv-bridge
```

Training input:

```text
training/marker_images/
```

Put marker photos there from different angles, distances, and lighting. The
trainer tries to crop the marker region before extracting ORB features. Then run:

```bash
cd ~/ugv_project/ros2_ws
python3 src/ugv_perception/ugv_perception/marker_model_trainer.py \
  --image-dir src/ugv_perception/training/marker_images \
  --model-out src/ugv_perception/models/marker_orb_model.npz \
  --max-descriptors 65000
```

The direct Python command is preferred on the Jetson when another underlay also
contains `ugv_perception`.

Run through the normal Jetson launcher:

```bash
START_MARKER_VISION=true MOTOR_DRY_RUN=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Standalone live CV test, with no motor/nav/LiDAR stack:

```bash
cd ~/ugv_project/ros2_ws
MARKER_VISION_TEST=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

This starts ZED image/depth publishing, `marker_vision_node`, and a small
terminal reporter. Expected output looks like:

```text
SEARCHING no marker yet frames=42 detections=0 last_reason=no_marker_candidate
MARKER DETECTED method=generic_dark_light_marker frames=4/53 distance=1.82m bearing=-6.2deg center=(391,442)px
```

If it prints `not_published_reason=stale_pose_or_missing`, CV is still seeing
the marker; it simply has no navigation pose because this standalone test does
not run nav. That is fine for checking detection accuracy and depth distance.

Topics:

- subscribes to `/zed/image`, `/zed/depth`, and `/ugv_nav_status`
- publishes confirmed marker points to `/ugv/marker_detection`; `x/y` are
  map-frame target coordinates and `z` carries camera/depth distance in meters
- publishes JSON debug status on `/ugv/marker_vision_debug`

Navigation already consumes `/ugv/marker_detection` and publishes
`/ugv/uav_flag` for the ESP/UAV handoff. Round 1 and Round 2 use the depth
distance to request stop once the marker is inside the configured 1-yard target
radius. Round 3 loiters near the marker instead of stopping.

Useful launch environment variables:

- `MARKER_MODEL_PATH`
- `MARKER_MODEL_MAX_DESCRIPTORS`
- `MARKER_MIN_GOOD_MATCHES`
- `MARKER_CONFIRMATION_FRAMES`
- `MARKER_CONFIRMATION_RADIUS_M`
- `MARKER_ENABLE_GENERIC_DETECTOR`
- `MARKER_GENERIC_MIN_AREA_FRAC`
- `MARKER_GENERIC_MIN_CONTRAST`
- `MARKER_VISION_TEST`
- `MARKER_VISION_TEST_PERIOD_S`

## Kept vs removed

Kept:

- `ugv_perception/obstacle_warning.py`: main depth-warning node
- `ugv_perception/zed_obj_distance.py`: optional debug helper
- `ugv_perception/marker_model_trainer.py`: marker image training scaffold
- `ugv_perception/marker_vision_node.py`: marker detection publisher

Removed from the main package flow:

- old direct ZED SDK scripts for accelerometer, gyroscope, magnetometer
- `zed_utils.py`, which only supported those one-off scripts
