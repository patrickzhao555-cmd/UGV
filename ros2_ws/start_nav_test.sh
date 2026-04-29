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

GOAL_FRAME_ID="${GOAL_FRAME_ID:-map}"
GOAL_X="${GOAL_X:-1.0}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_Z="${GOAL_Z:-0.0}"
WAIT_SECONDS="${WAIT_SECONDS:-0.5}"
ECHO_NAV_CMD="${ECHO_NAV_CMD:-true}"
ECHO_MOTOR_STATUS="${ECHO_MOTOR_STATUS:-true}"
EXTRA_SETUP_BASH="${EXTRA_SETUP_BASH:-}"

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup_compat "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source_setup_compat "/opt/ros/humble/setup.bash"
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

sleep "${WAIT_SECONDS}"

ros2 topic pub --once /ugv_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: ${GOAL_FRAME_ID}}, point: {x: ${GOAL_X}, y: ${GOAL_Y}, z: ${GOAL_Z}}}"

if [[ "${ECHO_NAV_CMD}" == "true" ]]; then
  ros2 topic echo /ugv_nav_cmd --once
fi

if [[ "${ECHO_MOTOR_STATUS}" == "true" ]]; then
  ros2 topic echo /motor_controller/status --once
fi
