#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source_setup_compat() {
  local setup_path="$1"
  local restore_nounset=0
  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "${setup_path}"
  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
}

# Edit these defaults once for your Jetson if the device names change.
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyUSB0}"
MOTOR_PORT="${MOTOR_PORT:-/dev/ttyACM0}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
MOTOR_BAUD="${MOTOR_BAUD:-115200}"
MOTOR_RAW_COMMAND_SCALE_US="${MOTOR_RAW_COMMAND_SCALE_US:-900.0}"
MOTOR_COMMAND_TIMEOUT_S="${MOTOR_COMMAND_TIMEOUT_S:-3.0}"
MOTOR_COMMAND_REFRESH_PERIOD_S="${MOTOR_COMMAND_REFRESH_PERIOD_S:-0.25}"
START_UWB="${START_UWB:-false}"
START_ZED="${START_ZED:-true}"
START_LIDAR="${START_LIDAR:-true}"
START_FUSION="${START_FUSION:-true}"
START_MOTOR_CONTROLLER="${START_MOTOR_CONTROLLER:-true}"
START_NAV="${START_NAV:-true}"
START_DEBUG_STATUS="${START_DEBUG_STATUS:-true}"
BENCH_TEST="${BENCH_TEST:-false}"
START_BENCH_GOAL="${START_BENCH_GOAL:-false}"
START_MOCK_FIELD_MAP="${START_MOCK_FIELD_MAP:-false}"
START_MARKER_VISION_WAS_SET="${START_MARKER_VISION+x}"
START_MARKER_VISION="${START_MARKER_VISION:-true}"
MARKER_VISION_TEST="${MARKER_VISION_TEST:-false}"
MARKER_VISION_TEST_PERIOD_S="${MARKER_VISION_TEST_PERIOD_S:-3.0}"
START_YOLO_OBSTACLES="${START_YOLO_OBSTACLES:-false}"
START_DEBUG_DASHBOARD="${START_DEBUG_DASHBOARD:-false}"
ROUND_MODE="${ROUND_MODE:-manual}"
COMPETITION_MODE="${COMPETITION_MODE:-false}"
START_CORNER="${START_CORNER:-lower_left}"
UGV_START_X_M="${UGV_START_X_M:-nan}"
UGV_START_Y_M="${UGV_START_Y_M:-nan}"
UGV_START_YAW_DEG="${UGV_START_YAW_DEG:-nan}"
CENTER_LOITER_RADIUS_M="${CENTER_LOITER_RADIUS_M:-0.75}"
TARGET_ACCEPT_RADIUS_M="${TARGET_ACCEPT_RADIUS_M:-0.9144}"
MIN_SPEED_MPS="${MIN_SPEED_MPS:-0.178816}"
DRIVE_SPEED_LEVEL="${DRIVE_SPEED_LEVEL:-4}"
ROUND_STRAIGHT_DISTANCE_M="${ROUND_STRAIGHT_DISTANCE_M:-11.8872}"
COMPETITION_MISSION_V2_ENABLED="${COMPETITION_MISSION_V2_ENABLED:-true}"
SWEEP_CELL_SIZE_M="${SWEEP_CELL_SIZE_M:-0.75}"
SWEEP_LANE_SPACING_M="${SWEEP_LANE_SPACING_M:-0.75}"
SWEEP_COVERAGE_RADIUS_M="${SWEEP_COVERAGE_RADIUS_M:-0.55}"
SWEEP_COVERAGE_THRESHOLD="${SWEEP_COVERAGE_THRESHOLD:-0.85}"
SWEEP_GOAL_TIMEOUT_S="${SWEEP_GOAL_TIMEOUT_S:-8.0}"
SWEEP_FAIL_LIMIT="${SWEEP_FAIL_LIMIT:-3}"
MIN_COMPETITION_SPEED_MPS="${MIN_COMPETITION_SPEED_MPS:-0.0894}"
BENCH_GOAL_X_M="${BENCH_GOAL_X_M:-12.2}"
BENCH_GOAL_Y_M="${BENCH_GOAL_Y_M:-12.0}"
BENCH_GOAL_PERIOD_S="${BENCH_GOAL_PERIOD_S:-1.0}"
MOCK_MARKER_CELL="${MOCK_MARKER_CELL:-7,7}"
MOCK_OBSTACLES_JSON="${MOCK_OBSTACLES_JSON:-[]}"
TARGET_TOPIC="${TARGET_TOPIC:-/ugv/target}"
ZED_PUBLISH_RATE_HZ="${ZED_PUBLISH_RATE_HZ:-10.0}"
ZED_DEPTH_DOWNSAMPLE_FACTOR="${ZED_DEPTH_DOWNSAMPLE_FACTOR:-2}"
ZED_PUBLISH_IMAGE="${ZED_PUBLISH_IMAGE:-false}"
FUSION_ZED_FRESH_TIMEOUT_S="${FUSION_ZED_FRESH_TIMEOUT_S:-0.75}"
FUSION_ALLOW_LIDAR_ONLY="${FUSION_ALLOW_LIDAR_ONLY:-true}"
FUSION_DEPTH_INVALID_WARN_FRAMES="${FUSION_DEPTH_INVALID_WARN_FRAMES:-2}"
FUSION_LIDAR_FRONT_FOV_DEG="${FUSION_LIDAR_FRONT_FOV_DEG:-70.0}"
FUSION_DEPTH_PROJECTION_STRIDE_PX="${FUSION_DEPTH_PROJECTION_STRIDE_PX:-8}"
FUSION_DEPTH_GROUND_FILTER_ENABLED="${FUSION_DEPTH_GROUND_FILTER_ENABLED:-true}"
FUSION_DEPTH_GROUND_MIN_DELTA_M="${FUSION_DEPTH_GROUND_MIN_DELTA_M:-0.18}"
FUSION_DEPTH_GROUND_RATIO="${FUSION_DEPTH_GROUND_RATIO:-0.88}"
FUSION_DEPTH_OBSTACLE_MIN_COMPONENT_HEIGHT_PX="${FUSION_DEPTH_OBSTACLE_MIN_COMPONENT_HEIGHT_PX:-14}"
FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M="${FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M:-0.42}"
FUSION_IMU_SMOOTHING_ALPHA="${FUSION_IMU_SMOOTHING_ALPHA:-0.25}"
MISSION_FLAG_TOPIC="${MISSION_FLAG_TOPIC:-/ugv/mission_flag}"
USE_IMU_YAW="${USE_IMU_YAW:-false}"
IMU_YAW_BLEND="${IMU_YAW_BLEND:-0.25}"
IMU_YAW_AXIS="${IMU_YAW_AXIS:-z}"
IMU_YAW_SIGN="${IMU_YAW_SIGN:-1.0}"
ROBOT_LENGTH_M="${ROBOT_LENGTH_M:-0.762}"
ROBOT_WIDTH_M="${ROBOT_WIDTH_M:-0.762}"
ROBOT_TRACK_WIDTH_M="${ROBOT_TRACK_WIDTH_M:-0.6096}"
ROBOT_WHEEL_RADIUS_M="${ROBOT_WHEEL_RADIUS_M:-0.06}"
ROBOT_TICKS_PER_REV="${ROBOT_TICKS_PER_REV:-1000}"
ROBOT_OBSTACLE_BUFFER_M="${ROBOT_OBSTACLE_BUFFER_M:-0.025}"
LIDAR_OFFSET_X_M="${LIDAR_OFFSET_X_M:-0.30}"
LIDAR_OFFSET_Y_M="${LIDAR_OFFSET_Y_M:-0.0}"
LIDAR_USED_FOV_DEG="${LIDAR_USED_FOV_DEG:-180.0}"
NAV_ALLOW_REVERSE="${NAV_ALLOW_REVERSE:-false}"
NAV_FRONT_SAFETY_MARGIN_M="${NAV_FRONT_SAFETY_MARGIN_M:-0.08}"
NAV_REAR_SAFETY_MARGIN_M="${NAV_REAR_SAFETY_MARGIN_M:-0.08}"
NAV_LOCAL_PLAN_INFLATION_M="${NAV_LOCAL_PLAN_INFLATION_M:-0.0}"
NAV_ACTIVE_SCAN_ENABLED="${NAV_ACTIVE_SCAN_ENABLED:-true}"
NAV_ACTIVE_SCAN_CONFIRM_STEPS="${NAV_ACTIVE_SCAN_CONFIRM_STEPS:-3}"
NAV_ACTIVE_SCAN_STEPS="${NAV_ACTIVE_SCAN_STEPS:-7}"
NAV_ACTIVE_SCAN_PANORAMIC_STEPS="${NAV_ACTIVE_SCAN_PANORAMIC_STEPS:-20}"
NAV_ACTIVE_SCAN_COOLDOWN_STEPS="${NAV_ACTIVE_SCAN_COOLDOWN_STEPS:-4}"
NAV_ACTIVE_SCAN_PROBE_STEPS="${NAV_ACTIVE_SCAN_PROBE_STEPS:-5}"
NAV_ACTIVE_SCAN_FRONT_CLEAR_M="${NAV_ACTIVE_SCAN_FRONT_CLEAR_M:-1.25}"
NAV_ACTIVE_SCAN_CORRIDOR_EXTRA_WIDTH_M="${NAV_ACTIVE_SCAN_CORRIDOR_EXTRA_WIDTH_M:-0.03}"
NAV_CONTINUOUS_CONTROL_ENABLED="${NAV_CONTINUOUS_CONTROL_ENABLED:-true}"
NAV_CONTINUOUS_MAX_SPEED_MPS="${NAV_CONTINUOUS_MAX_SPEED_MPS:-0.36}"
NAV_CONTINUOUS_MAX_OMEGA_RPS="${NAV_CONTINUOUS_MAX_OMEGA_RPS:-1.15}"
NAV_CONTINUOUS_HORIZON_S="${NAV_CONTINUOUS_HORIZON_S:-1.35}"
NAV_CONTINUOUS_ACCEL_LIMIT_MPS2="${NAV_CONTINUOUS_ACCEL_LIMIT_MPS2:-0.35}"
NAV_CONTINUOUS_OMEGA_ACCEL_LIMIT_RPS2="${NAV_CONTINUOUS_OMEGA_ACCEL_LIMIT_RPS2:-1.80}"
NAV_CONTINUOUS_LOWPASS_ALPHA="${NAV_CONTINUOUS_LOWPASS_ALPHA:-0.55}"
NAV_CONTINUOUS_RAW_PER_MPS="${NAV_CONTINUOUS_RAW_PER_MPS:-1.35}"
NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M="${NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M:-1.20}"
NAV_CONTINUOUS_STOP_CLEARANCE_M="${NAV_CONTINUOUS_STOP_CLEARANCE_M:-0.48}"
NAV_CONTINUOUS_GAP_BUFFER_M="${NAV_CONTINUOUS_GAP_BUFFER_M:-0.025}"
NAV_CONTINUOUS_LATENCY_BUFFER_S="${NAV_CONTINUOUS_LATENCY_BUFFER_S:-0.25}"
NAV_CONTINUOUS_ALLOW_COSTMAP_SOFT_PENALTY="${NAV_CONTINUOUS_ALLOW_COSTMAP_SOFT_PENALTY:-false}"
NAV_EMIT_VELOCITY_COMMANDS="${NAV_EMIT_VELOCITY_COMMANDS:-true}"
NAV_LOCAL_COSTMAP_ENABLED="${NAV_LOCAL_COSTMAP_ENABLED:-true}"
NAV_LOCAL_COSTMAP_WIDTH_M="${NAV_LOCAL_COSTMAP_WIDTH_M:-4.0}"
NAV_LOCAL_COSTMAP_HEIGHT_M="${NAV_LOCAL_COSTMAP_HEIGHT_M:-4.0}"
NAV_LOCAL_COSTMAP_RESOLUTION_M="${NAV_LOCAL_COSTMAP_RESOLUTION_M:-0.06}"
NAV_LOCAL_COSTMAP_DYNAMIC_DECAY_S="${NAV_LOCAL_COSTMAP_DYNAMIC_DECAY_S:-1.0}"
NAV_LOCAL_COSTMAP_OBSTACLE_RADIUS_M="${NAV_LOCAL_COSTMAP_OBSTACLE_RADIUS_M:-0.06}"
NAV_LOCAL_COSTMAP_INFLATION_M="${NAV_LOCAL_COSTMAP_INFLATION_M:-0.08}"
NAV_LOCAL_COSTMAP_LIDAR_CLEAR_RADIUS_M="${NAV_LOCAL_COSTMAP_LIDAR_CLEAR_RADIUS_M:-0.05}"
NAV_LOCAL_COSTMAP_MAX_RAYTRACE_M="${NAV_LOCAL_COSTMAP_MAX_RAYTRACE_M:-4.0}"
MARKER_MODEL_PATH="${MARKER_MODEL_PATH:-${WORKSPACE_DIR}/src/ugv_perception/models/marker_orb_model.npz}"
MARKER_MODEL_MAX_DESCRIPTORS="${MARKER_MODEL_MAX_DESCRIPTORS:-65000}"
MARKER_MIN_GOOD_MATCHES="${MARKER_MIN_GOOD_MATCHES:-18}"
MARKER_CONFIRMATION_FRAMES="${MARKER_CONFIRMATION_FRAMES:-2}"
MARKER_CONFIRMATION_RADIUS_M="${MARKER_CONFIRMATION_RADIUS_M:-0.75}"
MARKER_SIZE_M="${MARKER_SIZE_M:-0.3048}"
MARKER_MIN_PROJECTED_SIZE_M="${MARKER_MIN_PROJECTED_SIZE_M:-0.10}"
MARKER_MAX_PROJECTED_SIZE_M="${MARKER_MAX_PROJECTED_SIZE_M:-0.75}"
MARKER_ENABLE_GENERIC_DETECTOR="${MARKER_ENABLE_GENERIC_DETECTOR:-false}"
MARKER_GENERIC_MIN_AREA_FRAC="${MARKER_GENERIC_MIN_AREA_FRAC:-0.002}"
MARKER_GENERIC_MIN_CONTRAST="${MARKER_GENERIC_MIN_CONTRAST:-55.0}"
MARKER_GENERIC_MIN_GRID_SCORE="${MARKER_GENERIC_MIN_GRID_SCORE:-0.18}"
MARKER_GENERIC_MIN_BORDER_LIGHT_RATIO="${MARKER_GENERIC_MIN_BORDER_LIGHT_RATIO:-0.12}"
YOLO_MODEL_PATH="${YOLO_MODEL_PATH:-yolov8n.pt}"
YOLO_DEVICE="${YOLO_DEVICE:-auto}"
YOLO_IMGSZ="${YOLO_IMGSZ:-416}"
YOLO_CONFIDENCE="${YOLO_CONFIDENCE:-0.35}"
YOLO_MAX_HZ="${YOLO_MAX_HZ:-2.0}"
YOLO_OBSTACLE_CLASSES="${YOLO_OBSTACLE_CLASSES:-person,chair,couch,dining table,bench,potted plant,backpack,suitcase}"
DASHBOARD_UPDATE_HZ="${DASHBOARD_UPDATE_HZ:-8.0}"
DASHBOARD_CAMERA_SEARCH_DEPTH_M="${DASHBOARD_CAMERA_SEARCH_DEPTH_M:-0.30}"
EXTRA_SETUP_BASH="${EXTRA_SETUP_BASH:-}"
MOTOR_DRY_RUN="${MOTOR_DRY_RUN:-false}"
MOTOR_PWM_SLEW_RATE_US_PER_S="${MOTOR_PWM_SLEW_RATE_US_PER_S:-2400.0}"
MOTOR_VELOCITY_CONTROL_ENABLED="${MOTOR_VELOCITY_CONTROL_ENABLED:-false}"
MOTOR_PREFER_VELOCITY_FIELDS="${MOTOR_PREFER_VELOCITY_FIELDS:-true}"
MOTOR_WHEEL_RADIUS_M="${MOTOR_WHEEL_RADIUS_M:-0.06}"
MOTOR_TICKS_PER_REV="${MOTOR_TICKS_PER_REV:-1000}"
MOTOR_VELOCITY_KP="${MOTOR_VELOCITY_KP:-0.80}"
MOTOR_VELOCITY_KI="${MOTOR_VELOCITY_KI:-0.0}"
MOTOR_VELOCITY_KD="${MOTOR_VELOCITY_KD:-0.02}"
MOTOR_VELOCITY_INTEGRAL_LIMIT="${MOTOR_VELOCITY_INTEGRAL_LIMIT:-0.30}"
MOTOR_VELOCITY_FEEDFORWARD_RAW_PER_MPS="${MOTOR_VELOCITY_FEEDFORWARD_RAW_PER_MPS:-1.35}"
MOTOR_VELOCITY_MIN_TARGET_MPS="${MOTOR_VELOCITY_MIN_TARGET_MPS:-0.02}"
MOTOR_VELOCITY_MAX_TARGET_MPS="${MOTOR_VELOCITY_MAX_TARGET_MPS:-0.60}"
MOTOR_VELOCITY_CONTROL_PERIOD_S="${MOTOR_VELOCITY_CONTROL_PERIOD_S:-0.05}"
MOTOR_VELOCITY_STALE_ENCODER_TIMEOUT_S="${MOTOR_VELOCITY_STALE_ENCODER_TIMEOUT_S:-0.25}"
MOTOR_VELOCITY_FALLBACK_TO_RAW_WITHOUT_ENCODER="${MOTOR_VELOCITY_FALLBACK_TO_RAW_WITHOUT_ENCODER:-false}"
MOTOR_VELOCITY_ENCODER_SPEED_FILTER_ALPHA="${MOTOR_VELOCITY_ENCODER_SPEED_FILTER_ALPHA:-0.65}"
MOTOR_VELOCITY_ENCODER_SPEED_MAX_MPS="${MOTOR_VELOCITY_ENCODER_SPEED_MAX_MPS:-2.0}"
MOTOR_VELOCITY_ENCODER_SPEED_MIN_DT_S="${MOTOR_VELOCITY_ENCODER_SPEED_MIN_DT_S:-0.015}"
MOTOR_VELOCITY_RAW_FALLBACK_FLOOR_ENABLED="${MOTOR_VELOCITY_RAW_FALLBACK_FLOOR_ENABLED:-false}"
MOTOR_VELOCITY_RAW_FALLBACK_MIN_WHEEL_RAW="${MOTOR_VELOCITY_RAW_FALLBACK_MIN_WHEEL_RAW:-0.14}"
MOTOR_VELOCITY_RAW_FALLBACK_MIN_TARGET_RAW="${MOTOR_VELOCITY_RAW_FALLBACK_MIN_TARGET_RAW:-0.001}"
MIN_MOTION_RAW="${MIN_MOTION_RAW:-0.22}"
# Current chassis wiring maps positive logical left raw to physical left-forward,
# while positive logical right raw needs inversion to physical right-forward.
# These defaults were confirmed with motor_direct_test on 2026-05-15.
INVERT_LEFT_COMMAND="${INVERT_LEFT_COMMAND:-false}"
INVERT_RIGHT_COMMAND="${INVERT_RIGHT_COMMAND:-true}"
INVERT_LEFT_ENCODER="${INVERT_LEFT_ENCODER:-false}"
INVERT_RIGHT_ENCODER="${INVERT_RIGHT_ENCODER:-false}"

if [[ "${BENCH_TEST}" == "true" ]]; then
  MOTOR_DRY_RUN=true
  START_DEBUG_STATUS=true
  START_BENCH_GOAL=true
fi

if [[ "${MARKER_VISION_TEST}" == "true" ]]; then
  START_MARKER_VISION=true
  START_NAV=false
  START_MOTOR_CONTROLLER=false
  START_DEBUG_STATUS=false
  START_BENCH_GOAL=false
  START_MOCK_FIELD_MAP=false
  START_LIDAR=false
  START_FUSION=false
  START_YOLO_OBSTACLES=false
fi

case "${ROUND_MODE}" in
  round1|r1|round_1|straight|straight_line)
    ROUND_MODE="round1"
    ;;
  round2|r2|round_2|uav_landing|marker_landing)
    ROUND_MODE="round2"
    ;;
  round3|r3|round_3|competition)
    ROUND_MODE="round3"
    ;;
  indoor|room|classroom|roomba|wander|indoor_search)
    ROUND_MODE="indoor"
    ;;
  manual|normal|manual_goal)
    ROUND_MODE="manual"
    ;;
  *)
    echo "Unsupported ROUND_MODE=${ROUND_MODE}. Use manual, indoor, round1, round2, or round3."
    exit 1
    ;;
esac

if [[ "${ROUND_MODE}" == "round3" ]]; then
  COMPETITION_MODE=true
fi

case "${DRIVE_SPEED_LEVEL}" in
  1|2|3|4)
    ;;
  *)
    echo "Unsupported DRIVE_SPEED_LEVEL=${DRIVE_SPEED_LEVEL}. Use 1, 2, 3, or 4."
    exit 1
    ;;
esac

if [[ -z "${START_MARKER_VISION_WAS_SET}" && "${ROUND_MODE}" =~ ^round[123]$ ]]; then
  START_MARKER_VISION=true
fi

if [[ "${START_MARKER_VISION}" == "true" || "${START_YOLO_OBSTACLES}" == "true" || "${START_DEBUG_DASHBOARD}" == "true" ]]; then
  ZED_PUBLISH_IMAGE=true
fi

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup_compat "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -n "${EXTRA_SETUP_BASH}" ]]; then
  if [[ ! -f "${EXTRA_SETUP_BASH}" ]]; then
    echo "Missing extra setup file: ${EXTRA_SETUP_BASH}"
    exit 1
  fi
  source_setup_compat "${EXTRA_SETUP_BASH}"
fi

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WORKSPACE_DIR}/install/setup.bash"
  echo "Build the workspace first:"
  echo "  cd ${WORKSPACE_DIR}"
  echo "  colcon build --symlink-install"
  exit 1
fi

source_setup_compat "${WORKSPACE_DIR}/install/setup.bash"
export UGV_WS="${WORKSPACE_DIR}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"

echo "UGV bringup workspace: ${WORKSPACE_DIR}"
echo "UGV bringup launch: ${WORKSPACE_DIR}/src/ugv_sensor_sync/launch/competition_bringup.launch.py"

ros2 launch "${WORKSPACE_DIR}/src/ugv_sensor_sync/launch/competition_bringup.launch.py" \
  start_uwb:="${START_UWB}" \
  start_zed:="${START_ZED}" \
  start_lidar:="${START_LIDAR}" \
  start_fusion:="${START_FUSION}" \
  start_motor_controller:="${START_MOTOR_CONTROLLER}" \
  start_nav:="${START_NAV}" \
  start_debug_status:="${START_DEBUG_STATUS}" \
  start_bench_goal:="${START_BENCH_GOAL}" \
  start_mock_field_map:="${START_MOCK_FIELD_MAP}" \
  start_marker_vision:="${START_MARKER_VISION}" \
  start_marker_vision_test:="${MARKER_VISION_TEST}" \
  start_yolo_obstacles:="${START_YOLO_OBSTACLES}" \
  start_debug_dashboard:="${START_DEBUG_DASHBOARD}" \
  competition_mode:="${COMPETITION_MODE}" \
  mission_mode:="${ROUND_MODE}" \
  start_corner:="${START_CORNER}" \
  ugv_start_x_m:="${UGV_START_X_M}" \
  ugv_start_y_m:="${UGV_START_Y_M}" \
  ugv_start_yaw_deg:="${UGV_START_YAW_DEG}" \
  center_loiter_radius_m:="${CENTER_LOITER_RADIUS_M}" \
  target_accept_radius_m:="${TARGET_ACCEPT_RADIUS_M}" \
  min_speed_mps:="${MIN_SPEED_MPS}" \
  drive_speed_level:="${DRIVE_SPEED_LEVEL}" \
  straight_distance_m:="${ROUND_STRAIGHT_DISTANCE_M}" \
  competition_mission_v2_enabled:="${COMPETITION_MISSION_V2_ENABLED}" \
  sweep_cell_size_m:="${SWEEP_CELL_SIZE_M}" \
  sweep_lane_spacing_m:="${SWEEP_LANE_SPACING_M}" \
  sweep_coverage_radius_m:="${SWEEP_COVERAGE_RADIUS_M}" \
  sweep_coverage_threshold:="${SWEEP_COVERAGE_THRESHOLD}" \
  sweep_goal_timeout_s:="${SWEEP_GOAL_TIMEOUT_S}" \
  sweep_fail_limit:="${SWEEP_FAIL_LIMIT}" \
  min_competition_speed_mps:="${MIN_COMPETITION_SPEED_MPS}" \
  bench_goal_x_m:="${BENCH_GOAL_X_M}" \
  bench_goal_y_m:="${BENCH_GOAL_Y_M}" \
  bench_goal_period_s:="${BENCH_GOAL_PERIOD_S}" \
  mock_marker_cell:="${MOCK_MARKER_CELL}" \
  mock_obstacles_json:="${MOCK_OBSTACLES_JSON}" \
  target_topic:="${TARGET_TOPIC}" \
  marker_model_path:="${MARKER_MODEL_PATH}" \
  marker_model_max_descriptors:="${MARKER_MODEL_MAX_DESCRIPTORS}" \
  marker_min_good_matches:="${MARKER_MIN_GOOD_MATCHES}" \
  marker_confirmation_frames:="${MARKER_CONFIRMATION_FRAMES}" \
  marker_confirmation_radius_m:="${MARKER_CONFIRMATION_RADIUS_M}" \
  marker_size_m:="${MARKER_SIZE_M}" \
  marker_min_projected_size_m:="${MARKER_MIN_PROJECTED_SIZE_M}" \
  marker_max_projected_size_m:="${MARKER_MAX_PROJECTED_SIZE_M}" \
  marker_enable_generic_detector:="${MARKER_ENABLE_GENERIC_DETECTOR}" \
  marker_generic_min_area_frac:="${MARKER_GENERIC_MIN_AREA_FRAC}" \
  marker_generic_min_contrast:="${MARKER_GENERIC_MIN_CONTRAST}" \
  marker_generic_min_grid_score:="${MARKER_GENERIC_MIN_GRID_SCORE}" \
  marker_generic_min_border_light_ratio:="${MARKER_GENERIC_MIN_BORDER_LIGHT_RATIO}" \
  marker_vision_test_period_s:="${MARKER_VISION_TEST_PERIOD_S}" \
  yolo_model_path:="${YOLO_MODEL_PATH}" \
  yolo_device:="${YOLO_DEVICE}" \
  yolo_imgsz:="${YOLO_IMGSZ}" \
  yolo_confidence:="${YOLO_CONFIDENCE}" \
  yolo_max_hz:="${YOLO_MAX_HZ}" \
  yolo_obstacle_classes:="${YOLO_OBSTACLE_CLASSES}" \
  dashboard_update_hz:="${DASHBOARD_UPDATE_HZ}" \
  dashboard_camera_search_depth_m:="${DASHBOARD_CAMERA_SEARCH_DEPTH_M}" \
  lidar_port:="${LIDAR_PORT}" \
  lidar_baud:="${LIDAR_BAUD}" \
  zed_publish_rate_hz:="${ZED_PUBLISH_RATE_HZ}" \
  zed_depth_downsample_factor:="${ZED_DEPTH_DOWNSAMPLE_FACTOR}" \
  zed_publish_image:="${ZED_PUBLISH_IMAGE}" \
  fusion_zed_fresh_timeout_s:="${FUSION_ZED_FRESH_TIMEOUT_S}" \
  fusion_allow_lidar_only:="${FUSION_ALLOW_LIDAR_ONLY}" \
  fusion_depth_invalid_warn_frames:="${FUSION_DEPTH_INVALID_WARN_FRAMES}" \
  fusion_lidar_front_fov_deg:="${FUSION_LIDAR_FRONT_FOV_DEG}" \
  fusion_depth_projection_stride_px:="${FUSION_DEPTH_PROJECTION_STRIDE_PX}" \
  fusion_depth_ground_filter_enabled:="${FUSION_DEPTH_GROUND_FILTER_ENABLED}" \
  fusion_depth_ground_min_delta_m:="${FUSION_DEPTH_GROUND_MIN_DELTA_M}" \
  fusion_depth_ground_ratio:="${FUSION_DEPTH_GROUND_RATIO}" \
  fusion_depth_obstacle_min_component_height_px:="${FUSION_DEPTH_OBSTACLE_MIN_COMPONENT_HEIGHT_PX}" \
  fusion_depth_front_corridor_half_width_m:="${FUSION_DEPTH_FRONT_CORRIDOR_HALF_WIDTH_M}" \
  fusion_imu_smoothing_alpha:="${FUSION_IMU_SMOOTHING_ALPHA}" \
  mission_flag_topic:="${MISSION_FLAG_TOPIC}" \
  use_imu_yaw:="${USE_IMU_YAW}" \
  imu_yaw_blend:="${IMU_YAW_BLEND}" \
  imu_yaw_axis:="${IMU_YAW_AXIS}" \
  imu_yaw_sign:="${IMU_YAW_SIGN}" \
  robot_length_m:="${ROBOT_LENGTH_M}" \
  robot_width_m:="${ROBOT_WIDTH_M}" \
  robot_track_width_m:="${ROBOT_TRACK_WIDTH_M}" \
  robot_wheel_radius_m:="${ROBOT_WHEEL_RADIUS_M}" \
  robot_ticks_per_rev:="${ROBOT_TICKS_PER_REV}" \
  robot_obstacle_buffer_m:="${ROBOT_OBSTACLE_BUFFER_M}" \
  lidar_offset_x_m:="${LIDAR_OFFSET_X_M}" \
  lidar_offset_y_m:="${LIDAR_OFFSET_Y_M}" \
  lidar_used_fov_deg:="${LIDAR_USED_FOV_DEG}" \
  allow_reverse:="${NAV_ALLOW_REVERSE}" \
  front_safety_margin_m:="${NAV_FRONT_SAFETY_MARGIN_M}" \
  rear_safety_margin_m:="${NAV_REAR_SAFETY_MARGIN_M}" \
  local_plan_inflation_m:="${NAV_LOCAL_PLAN_INFLATION_M}" \
  active_scan_enabled:="${NAV_ACTIVE_SCAN_ENABLED}" \
  active_scan_confirm_steps:="${NAV_ACTIVE_SCAN_CONFIRM_STEPS}" \
  active_scan_steps:="${NAV_ACTIVE_SCAN_STEPS}" \
  active_scan_panoramic_steps:="${NAV_ACTIVE_SCAN_PANORAMIC_STEPS}" \
  active_scan_cooldown_steps:="${NAV_ACTIVE_SCAN_COOLDOWN_STEPS}" \
  active_scan_probe_steps:="${NAV_ACTIVE_SCAN_PROBE_STEPS}" \
  active_scan_front_clear_m:="${NAV_ACTIVE_SCAN_FRONT_CLEAR_M}" \
  active_scan_corridor_extra_width_m:="${NAV_ACTIVE_SCAN_CORRIDOR_EXTRA_WIDTH_M}" \
  continuous_control_enabled:="${NAV_CONTINUOUS_CONTROL_ENABLED}" \
  continuous_max_speed_mps:="${NAV_CONTINUOUS_MAX_SPEED_MPS}" \
  continuous_max_omega_rps:="${NAV_CONTINUOUS_MAX_OMEGA_RPS}" \
  continuous_horizon_s:="${NAV_CONTINUOUS_HORIZON_S}" \
  continuous_accel_limit_mps2:="${NAV_CONTINUOUS_ACCEL_LIMIT_MPS2}" \
  continuous_omega_accel_limit_rps2:="${NAV_CONTINUOUS_OMEGA_ACCEL_LIMIT_RPS2}" \
  continuous_lowpass_alpha:="${NAV_CONTINUOUS_LOWPASS_ALPHA}" \
  continuous_raw_per_mps:="${NAV_CONTINUOUS_RAW_PER_MPS}" \
  continuous_slowdown_clearance_m:="${NAV_CONTINUOUS_SLOWDOWN_CLEARANCE_M}" \
  continuous_stop_clearance_m:="${NAV_CONTINUOUS_STOP_CLEARANCE_M}" \
  continuous_gap_buffer_m:="${NAV_CONTINUOUS_GAP_BUFFER_M}" \
  continuous_latency_buffer_s:="${NAV_CONTINUOUS_LATENCY_BUFFER_S}" \
  continuous_allow_costmap_soft_penalty:="${NAV_CONTINUOUS_ALLOW_COSTMAP_SOFT_PENALTY}" \
  emit_velocity_commands:="${NAV_EMIT_VELOCITY_COMMANDS}" \
  local_costmap_enabled:="${NAV_LOCAL_COSTMAP_ENABLED}" \
  local_costmap_width_m:="${NAV_LOCAL_COSTMAP_WIDTH_M}" \
  local_costmap_height_m:="${NAV_LOCAL_COSTMAP_HEIGHT_M}" \
  local_costmap_resolution_m:="${NAV_LOCAL_COSTMAP_RESOLUTION_M}" \
  local_costmap_dynamic_decay_s:="${NAV_LOCAL_COSTMAP_DYNAMIC_DECAY_S}" \
  local_costmap_obstacle_radius_m:="${NAV_LOCAL_COSTMAP_OBSTACLE_RADIUS_M}" \
  local_costmap_inflation_m:="${NAV_LOCAL_COSTMAP_INFLATION_M}" \
  local_costmap_lidar_clear_radius_m:="${NAV_LOCAL_COSTMAP_LIDAR_CLEAR_RADIUS_M}" \
  local_costmap_max_raytrace_m:="${NAV_LOCAL_COSTMAP_MAX_RAYTRACE_M}" \
  motor_port:="${MOTOR_PORT}" \
  motor_baud:="${MOTOR_BAUD}" \
  motor_raw_command_scale_us:="${MOTOR_RAW_COMMAND_SCALE_US}" \
  motor_pwm_slew_rate_us_per_s:="${MOTOR_PWM_SLEW_RATE_US_PER_S}" \
  motor_command_timeout_s:="${MOTOR_COMMAND_TIMEOUT_S}" \
  motor_command_refresh_period_s:="${MOTOR_COMMAND_REFRESH_PERIOD_S}" \
  motor_dry_run:="${MOTOR_DRY_RUN}" \
  motor_velocity_control_enabled:="${MOTOR_VELOCITY_CONTROL_ENABLED}" \
  motor_prefer_velocity_fields:="${MOTOR_PREFER_VELOCITY_FIELDS}" \
  motor_wheel_radius_m:="${MOTOR_WHEEL_RADIUS_M}" \
  motor_ticks_per_rev:="${MOTOR_TICKS_PER_REV}" \
  motor_velocity_kp:="${MOTOR_VELOCITY_KP}" \
  motor_velocity_ki:="${MOTOR_VELOCITY_KI}" \
  motor_velocity_kd:="${MOTOR_VELOCITY_KD}" \
  motor_velocity_integral_limit:="${MOTOR_VELOCITY_INTEGRAL_LIMIT}" \
  motor_velocity_feedforward_raw_per_mps:="${MOTOR_VELOCITY_FEEDFORWARD_RAW_PER_MPS}" \
  motor_velocity_min_target_mps:="${MOTOR_VELOCITY_MIN_TARGET_MPS}" \
  motor_velocity_max_target_mps:="${MOTOR_VELOCITY_MAX_TARGET_MPS}" \
  motor_velocity_control_period_s:="${MOTOR_VELOCITY_CONTROL_PERIOD_S}" \
  motor_velocity_stale_encoder_timeout_s:="${MOTOR_VELOCITY_STALE_ENCODER_TIMEOUT_S}" \
  motor_velocity_fallback_to_raw_without_encoder:="${MOTOR_VELOCITY_FALLBACK_TO_RAW_WITHOUT_ENCODER}" \
  motor_velocity_encoder_speed_filter_alpha:="${MOTOR_VELOCITY_ENCODER_SPEED_FILTER_ALPHA}" \
  motor_velocity_encoder_speed_max_mps:="${MOTOR_VELOCITY_ENCODER_SPEED_MAX_MPS}" \
  motor_velocity_encoder_speed_min_dt_s:="${MOTOR_VELOCITY_ENCODER_SPEED_MIN_DT_S}" \
  motor_velocity_raw_fallback_floor_enabled:="${MOTOR_VELOCITY_RAW_FALLBACK_FLOOR_ENABLED}" \
  motor_velocity_raw_fallback_min_wheel_raw:="${MOTOR_VELOCITY_RAW_FALLBACK_MIN_WHEEL_RAW}" \
  motor_velocity_raw_fallback_min_target_raw:="${MOTOR_VELOCITY_RAW_FALLBACK_MIN_TARGET_RAW}" \
  min_motion_raw:="${MIN_MOTION_RAW}" \
  invert_left_command:="${INVERT_LEFT_COMMAND}" \
  invert_right_command:="${INVERT_RIGHT_COMMAND}" \
  invert_left_encoder:="${INVERT_LEFT_ENCODER}" \
  invert_right_encoder:="${INVERT_RIGHT_ENCODER}"
