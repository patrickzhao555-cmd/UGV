#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

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

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup_compat "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source_setup_compat "/opt/ros/humble/setup.bash"
fi

if [[ -n "${EXTRA_SETUP_BASH:-}" ]]; then
  if [[ ! -f "${EXTRA_SETUP_BASH}" ]]; then
    echo "Missing extra setup file: ${EXTRA_SETUP_BASH}"
    exit 1
  fi
  source_setup_compat "${EXTRA_SETUP_BASH}"
elif [[ -f "${HOME}/ugv_ws_albert/install/setup.bash" ]]; then
  source_setup_compat "${HOME}/ugv_ws_albert/install/setup.bash"
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source_setup_compat "${WORKSPACE_DIR}/install/setup.bash"
fi

MISSION_FLAG_PAYLOAD='{"state":"stop","source":"stop_ugv.sh"}'
STOP_CMD_PAYLOAD='{"mode":"STOP","command_type":"stop","reason":"manual stop_ugv.sh","v_mps":0.0,"omega_radps":0.0}'

if ! command -v timeout >/dev/null 2>&1; then
  echo "GNU timeout is required so stop publishes cannot block forever."
  exit 1
fi

echo "sending immediate motor stop"
timeout 1.2s ros2 topic pub -r 10 /ugv_nav_cmd std_msgs/msg/String "{data: '${STOP_CMD_PAYLOAD}'}" || true

echo "sending mission stop"
timeout 2s ros2 topic pub --once /ugv/mission_flag std_msgs/msg/String "{data: '${MISSION_FLAG_PAYLOAD}'}" || true

echo "sending final motor stop"
timeout 1.2s ros2 topic pub -r 10 /ugv_nav_cmd std_msgs/msg/String "{data: '${STOP_CMD_PAYLOAD}'}" || true

echo "Reading one /motor_controller/status sample, if available"
timeout 2s ros2 topic echo --once /motor_controller/status std_msgs/msg/String || true

echo "done"
