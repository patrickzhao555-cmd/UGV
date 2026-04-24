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
MOTOR_PORT="${MOTOR_PORT:-/dev/ttyTHS1}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
MOTOR_BAUD="${MOTOR_BAUD:-115200}"
MOTOR_RAW_COMMAND_SCALE_US="${MOTOR_RAW_COMMAND_SCALE_US:-900.0}"
START_UWB="${START_UWB:-false}"
START_MOTOR_CONTROLLER="${START_MOTOR_CONTROLLER:-true}"
START_NAV="${START_NAV:-true}"
EXTRA_SETUP_BASH="${EXTRA_SETUP_BASH:-}"
INVERT_LEFT_COMMAND="${INVERT_LEFT_COMMAND:-false}"
INVERT_RIGHT_COMMAND="${INVERT_RIGHT_COMMAND:-false}"
INVERT_LEFT_ENCODER="${INVERT_LEFT_ENCODER:-false}"
INVERT_RIGHT_ENCODER="${INVERT_RIGHT_ENCODER:-false}"

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
  lidar_port:="${LIDAR_PORT}" \
  lidar_baud:="${LIDAR_BAUD}" \
  motor_port:="${MOTOR_PORT}" \
  motor_baud:="${MOTOR_BAUD}" \
  motor_raw_command_scale_us:="${MOTOR_RAW_COMMAND_SCALE_US}" \
  invert_left_command:="${INVERT_LEFT_COMMAND}" \
  invert_right_command:="${INVERT_RIGHT_COMMAND}" \
  invert_left_encoder:="${INVERT_LEFT_ENCODER}" \
  invert_right_encoder:="${INVERT_RIGHT_ENCODER}"
