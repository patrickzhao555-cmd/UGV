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

export MOTOR_WHEEL_RADIUS_M="${MOTOR_WHEEL_RADIUS_M:-0.0889}"
export MOTOR_TICKS_PER_REV="${MOTOR_TICKS_PER_REV:-3200}"
export MOTOR_TRACK_WIDTH_M="${MOTOR_TRACK_WIDTH_M:-0.6096}"
export MOTOR_TEENSY_PID_KP="${MOTOR_TEENSY_PID_KP:-0.80}"
export MOTOR_TEENSY_PID_KI="${MOTOR_TEENSY_PID_KI:-0.0}"
export MOTOR_TEENSY_PID_KD="${MOTOR_TEENSY_PID_KD:-0.02}"
export MOTOR_TEENSY_CONTROL_HZ="${MOTOR_TEENSY_CONTROL_HZ:-100.0}"
export MOTOR_TEENSY_SIDE_MISMATCH_FAULT_ENABLED="${MOTOR_TEENSY_SIDE_MISMATCH_FAULT_ENABLED:-true}"
export MOTOR_TEENSY_SIDE_MISMATCH_WARN_TPS="${MOTOR_TEENSY_SIDE_MISMATCH_WARN_TPS:-80.0}"
export MOTOR_TEENSY_SIDE_MISMATCH_FAULT_TPS="${MOTOR_TEENSY_SIDE_MISMATCH_FAULT_TPS:-180.0}"
export MOTOR_TEENSY_ENCODER_JUMP_FAULT_ENABLED="${MOTOR_TEENSY_ENCODER_JUMP_FAULT_ENABLED:-true}"
export MOTOR_TEENSY_ENCODER_JUMP_TPS="${MOTOR_TEENSY_ENCODER_JUMP_TPS:-12000.0}"

echo "Motor model: track_width_m=${MOTOR_TRACK_WIDTH_M}, wheel_radius_m=${MOTOR_WHEEL_RADIUS_M}, ticks_per_rev=${MOTOR_TICKS_PER_REV}"
echo "Teensy PID: kp=${MOTOR_TEENSY_PID_KP}, ki=${MOTOR_TEENSY_PID_KI}, kd=${MOTOR_TEENSY_PID_KD}, control_hz=${MOTOR_TEENSY_CONTROL_HZ}"
echo "Diagnostics: side_mismatch_warn_tps=${MOTOR_TEENSY_SIDE_MISMATCH_WARN_TPS}, side_mismatch_fault_tps=${MOTOR_TEENSY_SIDE_MISMATCH_FAULT_TPS}, encoder_jump_tps=${MOTOR_TEENSY_ENCODER_JUMP_TPS}"

exec ros2 launch ugv_motor_controller motor_controller.launch.py \
  track_width_m:="${MOTOR_TRACK_WIDTH_M}" \
  wheel_radius_m:="${MOTOR_WHEEL_RADIUS_M}" \
  ticks_per_rev:="${MOTOR_TICKS_PER_REV}" \
  teensy_control_hz:="${MOTOR_TEENSY_CONTROL_HZ}" \
  teensy_pid_kp:="${MOTOR_TEENSY_PID_KP}" \
  teensy_pid_ki:="${MOTOR_TEENSY_PID_KI}" \
  teensy_pid_kd:="${MOTOR_TEENSY_PID_KD}" \
  port:="${MOTOR_PORT:-/dev/ttyACM0}" \
  baud:="${MOTOR_BAUD:-115200}" \
  dry_run:="${MOTOR_DRY_RUN:-false}" \
  command_timeout_s:="${MOTOR_COMMAND_TIMEOUT_S:-0.75}" \
  teensy_left_motor_sign:="${MOTOR_TEENSY_LEFT_MOTOR_SIGN:-1}" \
  teensy_right_motor_sign:="${MOTOR_TEENSY_RIGHT_MOTOR_SIGN:--1}" \
  teensy_fl_encoder_sign:="${MOTOR_TEENSY_FL_ENCODER_SIGN:-1}" \
  teensy_fr_encoder_sign:="${MOTOR_TEENSY_FR_ENCODER_SIGN:-1}" \
  teensy_rl_encoder_sign:="${MOTOR_TEENSY_RL_ENCODER_SIGN:-1}" \
  teensy_rr_encoder_sign:="${MOTOR_TEENSY_RR_ENCODER_SIGN:-1}" \
  teensy_side_mismatch_fault_enabled:="${MOTOR_TEENSY_SIDE_MISMATCH_FAULT_ENABLED}" \
  teensy_side_mismatch_warn_tps:="${MOTOR_TEENSY_SIDE_MISMATCH_WARN_TPS}" \
  teensy_side_mismatch_fault_tps:="${MOTOR_TEENSY_SIDE_MISMATCH_FAULT_TPS}" \
  teensy_encoder_jump_fault_enabled:="${MOTOR_TEENSY_ENCODER_JUMP_FAULT_ENABLED}" \
  teensy_encoder_jump_tps:="${MOTOR_TEENSY_ENCODER_JUMP_TPS}"
