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
START_UWB="${START_UWB:-false}"
START_MOTOR_CONTROLLER="${START_MOTOR_CONTROLLER:-true}"
START_NAV="${START_NAV:-true}"
START_DEBUG_STATUS="${START_DEBUG_STATUS:-true}"
BENCH_TEST="${BENCH_TEST:-false}"
START_BENCH_GOAL="${START_BENCH_GOAL:-false}"
START_MOCK_FIELD_MAP="${START_MOCK_FIELD_MAP:-false}"
COMPETITION_MODE="${COMPETITION_MODE:-false}"
START_CORNER="${START_CORNER:-lower_left}"
CENTER_LOITER_RADIUS_M="${CENTER_LOITER_RADIUS_M:-0.75}"
BENCH_GOAL_X_M="${BENCH_GOAL_X_M:-12.2}"
BENCH_GOAL_Y_M="${BENCH_GOAL_Y_M:-12.0}"
BENCH_GOAL_PERIOD_S="${BENCH_GOAL_PERIOD_S:-1.0}"
MOCK_MARKER_CELL="${MOCK_MARKER_CELL:-7,7}"
MOCK_OBSTACLES_JSON="${MOCK_OBSTACLES_JSON:-[]}"
ZED_PUBLISH_RATE_HZ="${ZED_PUBLISH_RATE_HZ:-10.0}"
ZED_DEPTH_DOWNSAMPLE_FACTOR="${ZED_DEPTH_DOWNSAMPLE_FACTOR:-2}"
FUSION_ZED_FRESH_TIMEOUT_S="${FUSION_ZED_FRESH_TIMEOUT_S:-0.75}"
FUSION_DEPTH_INVALID_WARN_FRAMES="${FUSION_DEPTH_INVALID_WARN_FRAMES:-2}"
FUSION_LIDAR_FRONT_FOV_DEG="${FUSION_LIDAR_FRONT_FOV_DEG:-70.0}"
EXTRA_SETUP_BASH="${EXTRA_SETUP_BASH:-}"
MOTOR_DRY_RUN="${MOTOR_DRY_RUN:-false}"
INVERT_LEFT_COMMAND="${INVERT_LEFT_COMMAND:-false}"
INVERT_RIGHT_COMMAND="${INVERT_RIGHT_COMMAND:-false}"
INVERT_LEFT_ENCODER="${INVERT_LEFT_ENCODER:-false}"
INVERT_RIGHT_ENCODER="${INVERT_RIGHT_ENCODER:-false}"

if [[ "${BENCH_TEST}" == "true" ]]; then
  MOTOR_DRY_RUN=true
  START_DEBUG_STATUS=true
  START_BENCH_GOAL=true
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

ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  start_uwb:="${START_UWB}" \
  start_motor_controller:="${START_MOTOR_CONTROLLER}" \
  start_nav:="${START_NAV}" \
  start_debug_status:="${START_DEBUG_STATUS}" \
  start_bench_goal:="${START_BENCH_GOAL}" \
  start_mock_field_map:="${START_MOCK_FIELD_MAP}" \
  competition_mode:="${COMPETITION_MODE}" \
  start_corner:="${START_CORNER}" \
  center_loiter_radius_m:="${CENTER_LOITER_RADIUS_M}" \
  bench_goal_x_m:="${BENCH_GOAL_X_M}" \
  bench_goal_y_m:="${BENCH_GOAL_Y_M}" \
  bench_goal_period_s:="${BENCH_GOAL_PERIOD_S}" \
  mock_marker_cell:="${MOCK_MARKER_CELL}" \
  mock_obstacles_json:="${MOCK_OBSTACLES_JSON}" \
  lidar_port:="${LIDAR_PORT}" \
  lidar_baud:="${LIDAR_BAUD}" \
  zed_publish_rate_hz:="${ZED_PUBLISH_RATE_HZ}" \
  zed_depth_downsample_factor:="${ZED_DEPTH_DOWNSAMPLE_FACTOR}" \
  fusion_zed_fresh_timeout_s:="${FUSION_ZED_FRESH_TIMEOUT_S}" \
  fusion_depth_invalid_warn_frames:="${FUSION_DEPTH_INVALID_WARN_FRAMES}" \
  fusion_lidar_front_fov_deg:="${FUSION_LIDAR_FRONT_FOV_DEG}" \
  motor_port:="${MOTOR_PORT}" \
  motor_baud:="${MOTOR_BAUD}" \
  motor_raw_command_scale_us:="${MOTOR_RAW_COMMAND_SCALE_US}" \
  motor_dry_run:="${MOTOR_DRY_RUN}" \
  invert_left_command:="${INVERT_LEFT_COMMAND}" \
  invert_right_command:="${INVERT_RIGHT_COMMAND}" \
  invert_left_encoder:="${INVERT_LEFT_ENCODER}" \
  invert_right_encoder:="${INVERT_RIGHT_ENCODER}"
