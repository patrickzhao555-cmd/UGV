#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "--yes" ]]; then
  cat <<'EOF'
WHEELS OFF GROUND REQUIRED.

This starts only the motor controller bridge in Teensy side PID mode and may
allow motor commands from /ugv_nav_cmd once the Teensy ACKs every startup
parameter. Put the robot on a stable stand before running.

Run again with:
  scripts/run_teensy_side_pid_bench.sh --yes
EOF
  exit 2
fi

echo "WHEELS OFF GROUND REQUIRED: starting Teensy side PID bench bridge only."

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}/ros2_ws"

if [[ -f "install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "install/setup.bash"
fi

export MOTOR_CONTROL_LOCATION=teensy_pid
export MOTOR_WHEEL_RADIUS_M="${MOTOR_WHEEL_RADIUS_M:-0.0889}"
export MOTOR_TICKS_PER_REV="${MOTOR_TICKS_PER_REV:-3200}"

exec ros2 launch ugv_motor_controller motor_controller.launch.py \
  motor_control_location:="${MOTOR_CONTROL_LOCATION}" \
  wheel_radius_m:="${MOTOR_WHEEL_RADIUS_M}" \
  ticks_per_rev:="${MOTOR_TICKS_PER_REV}" \
  velocity_control_enabled:=false \
  port:="${MOTOR_PORT:-/dev/ttyACM0}" \
  baud:="${MOTOR_BAUD:-115200}" \
  dry_run:="${MOTOR_DRY_RUN:-false}" \
  command_timeout_s:="${MOTOR_COMMAND_TIMEOUT_S:-0.75}" \
  teensy_pid_param_ack_timeout_s:="${MOTOR_TEENSY_PARAM_ACK_TIMEOUT_S:-1.0}" \
  teensy_left_motor_sign:="${MOTOR_TEENSY_LEFT_MOTOR_SIGN:-1}" \
  teensy_right_motor_sign:="${MOTOR_TEENSY_RIGHT_MOTOR_SIGN:--1}" \
  teensy_fl_encoder_sign:="${MOTOR_TEENSY_FL_ENCODER_SIGN:-1}" \
  teensy_fr_encoder_sign:="${MOTOR_TEENSY_FR_ENCODER_SIGN:-1}" \
  teensy_rl_encoder_sign:="${MOTOR_TEENSY_RL_ENCODER_SIGN:-1}" \
  teensy_rr_encoder_sign:="${MOTOR_TEENSY_RR_ENCODER_SIGN:-1}"
