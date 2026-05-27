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

first_existing_port() {
  for port in "$@"; do
    if [[ -e "${port}" ]]; then
      echo "${port}"
      return 0
    fi
  done
  return 1
}

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup_compat "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source_setup_compat "/opt/ros/humble/setup.bash"
fi

if [[ -n "${EXTRA_SETUP_BASH:-}" ]]; then
  source_setup_compat "${EXTRA_SETUP_BASH}"
fi

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WORKSPACE_DIR}/install/setup.bash"
  echo "Build first: cd ${WORKSPACE_DIR} && colcon build --symlink-install"
  exit 1
fi
source_setup_compat "${WORKSPACE_DIR}/install/setup.bash"

MOTOR_PORT="${MOTOR_PORT:-$(first_existing_port /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1 || true)}"
if [[ -z "${MOTOR_PORT}" ]]; then
  echo "Could not auto-detect MOTOR_PORT. Set MOTOR_PORT=/dev/ttyACM0 or the correct device."
  exit 1
fi

MOTOR_WHEEL_RADIUS_M="${MOTOR_WHEEL_RADIUS_M:-0.0889}"
MOTOR_TICKS_PER_REV="${MOTOR_TICKS_PER_REV:-3200}"
MOTOR_DRY_RUN="${MOTOR_DRY_RUN:-false}"
START_NAV="${START_NAV:-true}"
START_MOTOR_CONTROLLER="${START_MOTOR_CONTROLLER:-true}"

echo "UGV clean runtime branch: cleanup/two-side-pid-runtime"
echo "Hardware: four Pololu motors, two goBILDA speed controllers"
echo "Motor control: Teensy two-side PID only"
echo "Motor model: wheel_radius_m=${MOTOR_WHEEL_RADIUS_M}, ticks_per_rev=${MOTOR_TICKS_PER_REV}"
echo "Motor port: ${MOTOR_PORT}"
echo "Navigation: STOP-only placeholder until new Jetson high-level nav is written"

exec ros2 launch ugv_sensor_sync competition_bringup.launch.py \
  start_motor_controller:="${START_MOTOR_CONTROLLER}" \
  start_nav:="${START_NAV}" \
  motor_port:="${MOTOR_PORT}" \
  motor_dry_run:="${MOTOR_DRY_RUN}" \
  motor_wheel_radius_m:="${MOTOR_WHEEL_RADIUS_M}" \
  motor_ticks_per_rev:="${MOTOR_TICKS_PER_REV}"
