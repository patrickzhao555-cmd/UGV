# ugv_perception

Perception and visual debugging package.

For full pull/build/launch instructions, use the consolidated runbooks:

- English: `ros2_ws/JETSON_BRINGUP_CHECKLIST.md`
- Chinese: `ros2_ws/JETSON_BRINGUP_CHECKLIST_ZH.md`

## Nodes

- `marker_vision_node`: ZED image/depth marker detection, publishes `/ugv/marker_detection` and `/ugv/marker_vision_debug`
- `marker_vision_test_node`: standalone live marker vision reporter
- `marker_model_trainer`: trains the lightweight ORB model from marker photos
- `yolo_semantic_obstacle_node`: optional YOLO semantic obstacle inflation points
- `ugv_debug_dashboard`: OpenCV/VNC dashboard for ZED image, YOLO boxes, marker debug, depth, LiDAR/fused points, session map, and nav status
- `obstacle_warning`: legacy depth warning helper
- `zed_obj_distance`: optional ZED object debug helper

## Marker Training

```bash
cd ~/ugv_project/ros2_ws
python3 src/ugv_perception/ugv_perception/marker_model_trainer.py \
  --image-dir src/ugv_perception/training/marker_images \
  --model-out src/ugv_perception/models/marker_orb_model.npz \
  --max-descriptors 65000
```

## YOLO Assist

YOLO is optional and advisory. It adds semantic obstacle points for conservative
inflation. LiDAR and ZED depth remain the hard collision sources.

```bash
python3 -m pip install --user --force-reinstall "numpy==1.26.4"
python3 -m pip install --user "ultralytics" "numpy<2"
```

Launch through the main stack:

```bash
START_YOLO_OBSTACLES=true EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

## Dashboard

The dashboard needs a GUI display, usually TigerVNC:

```bash
START_DEBUG_DASHBOARD=true START_YOLO_OBSTACLES=true \
EXTRA_SETUP_BASH=~/ugv_ws_albert/install/setup.bash bash jetson_bringup.sh
```

Keys:

- `s`: save screenshot
- `q` or `Esc`: close
