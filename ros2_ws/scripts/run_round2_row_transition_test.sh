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

first_existing_port() {
  for port in "$@"; do
    if [[ -e "${port}" ]]; then
      echo "${port}"
      return 0
    fi
  done
  return 1
}

first_matching_port() {
  local pattern
  for pattern in "$@"; do
    for port in ${pattern}; do
      if [[ -e "${port}" ]]; then
        echo "${port}"
        return 0
      fi
    done
  done
  return 1
}

export UGV_PROFILE="${UGV_PROFILE:-round2_row_transition_test}"
export MOTOR_PORT="${MOTOR_PORT:-$(first_matching_port '/dev/serial/by-id/*Teensyduino*' '/dev/serial/by-id/*Teensy*' || first_existing_port /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB1 /dev/ttyUSB0 || true)}"
export LIDAR_PORT="${LIDAR_PORT:-$(first_matching_port '/dev/serial/by-id/*Silicon_Labs*CP210*' '/dev/serial/by-id/*CP2102*' '/dev/serial/by-id/*CP210x*' '/dev/serial/by-id/*Silicon_Labs*' || first_existing_port /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1 || true)}"

if [[ -z "${MOTOR_PORT}" ]]; then
  echo "Could not auto-detect MOTOR_PORT. Set MOTOR_PORT=/dev/ttyACM0 or the correct device."
  exit 1
fi
if [[ -z "${LIDAR_PORT}" ]]; then
  echo "Could not auto-detect LIDAR_PORT. Set LIDAR_PORT=/dev/ttyUSB0 or the correct device."
  exit 1
fi

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

exec bash "${WORKSPACE_DIR}/jetson_bringup.sh"
