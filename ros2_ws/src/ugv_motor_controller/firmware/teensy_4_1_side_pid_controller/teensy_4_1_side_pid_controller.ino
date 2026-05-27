// Teensy 4.1 two-controller, four-encoder closed-loop motor controller.
//
// Hardware truth:
//   Four Pololu 50:1 37D motors with 64 CPR motor-shaft quadrature encoders.
//   Two goBILDA 1x15A R/C PWM brushed DC speed controllers.
//   PWM_L drives both left-side motors through one controller.
//   PWM_R drives both right-side motors through one controller.
//
// Four encoders are read independently. FL/RL are averaged for the left side
// PID, FR/RR are averaged for the right side PID, and same-side mismatch is
// diagnostics/fault detection only. The hardware has two actuator outputs, so
// it cannot independently correct front-vs-rear speed on the same side.
//
// Jetson -> Teensy:
//   CMD V <v_mps> <omega_radps>
//   CMD STOP
//   CMD RAW2 <left_us> <right_us>
//   CMD PARAM <name> <value>
//   CMD STATUS
//
// Teensy -> Jetson:
//   PARAM,<name>,ok
//   PARAM,<name>,unknown
//   PARAMS,<name>=<value>,...
//   S,<millis>,<fl_ticks>,<fr_ticks>,<rl_ticks>,<rr_ticks>,
//     <fl_tps>,<fr_tps>,<rl_tps>,<rr_tps>,
//     <left_target_tps>,<right_target_tps>,
//     <left_measured_tps>,<right_measured_tps>,
//     <left_pwm>,<right_pwm>,
//     <left_error>,<right_error>,
//     <left_p>,<left_i>,<left_d>,
//     <right_p>,<right_i>,<right_d>,
//     <fault>

#include <Arduino.h>
#include <Encoder.h>
#include <QuickPID.h>
#include <Servo.h>

#define ENC_REAR_LEFT_A    24
#define ENC_REAR_LEFT_B    25
#define ENC_REAR_RIGHT_A   26
#define ENC_REAR_RIGHT_B   27
#define ENC_FRONT_LEFT_A   38
#define ENC_FRONT_LEFT_B   39
#define ENC_FRONT_RIGHT_A  36
#define ENC_FRONT_RIGHT_B  37
#define PWM_L              34
#define PWM_R              35

const unsigned long DEFAULT_CONTROL_INTERVAL_MS = 10;  // 100 Hz
const unsigned long STATUS_INTERVAL_MS = 50;   // 20 Hz
const unsigned long DEFAULT_COMMAND_TIMEOUT_MS = 500;

const int DEFAULT_PWM_MIN_US = 1100;
const int DEFAULT_PWM_NEUTRAL_US = 1500;
const int DEFAULT_PWM_MAX_US = 1900;

const float DEFAULT_TRACK_WIDTH_M = 0.425f;
const float DEFAULT_WHEEL_RADIUS_M = 0.0825f;
const float DEFAULT_TICKS_PER_REV = 3200.0f;
const float DEFAULT_KP = 0.55f;
const float DEFAULT_KI = 0.0f;
const float DEFAULT_KD = 0.0f;

enum ControllerMode {
  MODE_STOPPED,
  MODE_VELOCITY,
  MODE_RAW2,
  MODE_FAULT
};

Encoder enc_fl(ENC_FRONT_LEFT_A, ENC_FRONT_LEFT_B);
Encoder enc_fr(ENC_FRONT_RIGHT_A, ENC_FRONT_RIGHT_B);
Encoder enc_rl(ENC_REAR_LEFT_A, ENC_REAR_LEFT_B);
Encoder enc_rr(ENC_REAR_RIGHT_A, ENC_REAR_RIGHT_B);

Servo motor_left;
Servo motor_right;

ControllerMode controller_mode = MODE_STOPPED;

unsigned long last_control_ms = 0;
unsigned long last_status_ms = 0;
unsigned long last_command_ms = 0;
unsigned long control_interval_ms = DEFAULT_CONTROL_INTERVAL_MS;
bool status_stream_enabled = true;
unsigned long fl_stall_start_ms = 0;
unsigned long rl_stall_start_ms = 0;
unsigned long fr_stall_start_ms = 0;
unsigned long rr_stall_start_ms = 0;
unsigned long left_side_stall_start_ms = 0;
unsigned long right_side_stall_start_ms = 0;
unsigned long left_mismatch_start_ms = 0;
unsigned long right_mismatch_start_ms = 0;
unsigned long fl_sign_start_ms = 0;
unsigned long rl_sign_start_ms = 0;
unsigned long fr_sign_start_ms = 0;
unsigned long rr_sign_start_ms = 0;

char usb_buf[96];
char uart_buf[96];
int usb_buf_idx = 0;
int uart_buf_idx = 0;

int pwm_min_us = DEFAULT_PWM_MIN_US;
int pwm_neutral_us = DEFAULT_PWM_NEUTRAL_US;
int pwm_max_us = DEFAULT_PWM_MAX_US;
int current_left_pwm = DEFAULT_PWM_NEUTRAL_US;
int current_right_pwm = DEFAULT_PWM_NEUTRAL_US;
int target_left_pwm = DEFAULT_PWM_NEUTRAL_US;
int target_right_pwm = DEFAULT_PWM_NEUTRAL_US;

float track_width_m = DEFAULT_TRACK_WIDTH_M;
float wheel_radius_m = DEFAULT_WHEEL_RADIUS_M;
float ticks_per_rev = DEFAULT_TICKS_PER_REV;
float command_timeout_ms = DEFAULT_COMMAND_TIMEOUT_MS;

float kp = DEFAULT_KP;
float ki = DEFAULT_KI;
float kd = DEFAULT_KD;
float feedforward_us_per_tps = 0.0f;
float pid_output_limit_us = 350.0f;
float pwm_slew_us_per_s = 2400.0f;
float min_target_tps = 2.0f;
float deadband_tps = 1.0f;

float target_v_mps = 0.0f;
float target_omega_radps = 0.0f;
float left_target_tps = 0.0f;
float right_target_tps = 0.0f;
float left_measured_tps = 0.0f;
float right_measured_tps = 0.0f;
float fl_tps = 0.0f;
float fr_tps = 0.0f;
float rl_tps = 0.0f;
float rr_tps = 0.0f;
float left_error_tps = 0.0f;
float right_error_tps = 0.0f;
float left_pid_output_us = 0.0f;
float right_pid_output_us = 0.0f;
float left_p_term = 0.0f;
float left_i_term = 0.0f;
float left_d_term = 0.0f;
float right_p_term = 0.0f;
float right_i_term = 0.0f;
float right_d_term = 0.0f;

long fl_ticks = 0;
long fr_ticks = 0;
long rl_ticks = 0;
long rr_ticks = 0;
long last_fl_ticks = 0;
long last_fr_ticks = 0;
long last_rl_ticks = 0;
long last_rr_ticks = 0;

int fl_encoder_sign = -1;
int fr_encoder_sign = 1;
int rl_encoder_sign = -1;
int rr_encoder_sign = 1;
int left_motor_sign = 1;
int right_motor_sign = -1;

bool stall_fault_enabled = true;
float stall_target_tps = 15.0f;
float stall_near_zero_tps = 2.0f;
float stall_moving_peer_tps = 12.0f;
float stall_pwm_delta_us = 120.0f;
unsigned long stall_timeout_ms = 300;
float sign_mismatch_tps = 10.0f;
float sign_mismatch_target_tps = 100.0f;
unsigned long sign_mismatch_timeout_ms = 250;
bool side_mismatch_fault_enabled = true;
float side_mismatch_warn_tps = 80.0f;
float side_mismatch_fault_tps = 180.0f;
bool encoder_jump_fault_enabled = true;
float encoder_jump_tps = 12000.0f;

char fault_reason[32] = "none";

float left_pid_input = 0.0f;
float right_pid_input = 0.0f;
float left_pid_setpoint = 0.0f;
float right_pid_setpoint = 0.0f;

QuickPID left_pid(
  &left_pid_input,
  &left_pid_output_us,
  &left_pid_setpoint,
  DEFAULT_KP,
  DEFAULT_KI,
  DEFAULT_KD,
  QuickPID::pMode::pOnError,
  QuickPID::dMode::dOnMeas,
  QuickPID::iAwMode::iAwClamp,
  QuickPID::Action::direct
);
QuickPID right_pid(
  &right_pid_input,
  &right_pid_output_us,
  &right_pid_setpoint,
  DEFAULT_KP,
  DEFAULT_KI,
  DEFAULT_KD,
  QuickPID::pMode::pOnError,
  QuickPID::dMode::dOnMeas,
  QuickPID::iAwMode::iAwClamp,
  QuickPID::Action::direct
);

float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

int clampPwm(int pwm) {
  return constrain(pwm, pwm_min_us, pwm_max_us);
}

int signOf(float value, float threshold) {
  if (value > threshold) return 1;
  if (value < -threshold) return -1;
  return 0;
}

float mpsToTicksPerSec(float mps) {
  const float circumference_m = 2.0f * PI * max(0.0001f, wheel_radius_m);
  return mps * max(1.0f, ticks_per_rev) / circumference_m;
}

long readEncoderSigned(Encoder& encoder, int sign) {
  return encoder.read() * sign;
}

void clearFault() {
  strncpy(fault_reason, "none", sizeof(fault_reason));
  fault_reason[sizeof(fault_reason) - 1] = '\0';
  fl_stall_start_ms = 0;
  rl_stall_start_ms = 0;
  fr_stall_start_ms = 0;
  rr_stall_start_ms = 0;
  left_side_stall_start_ms = 0;
  right_side_stall_start_ms = 0;
  left_mismatch_start_ms = 0;
  right_mismatch_start_ms = 0;
  fl_sign_start_ms = 0;
  rl_sign_start_ms = 0;
  fr_sign_start_ms = 0;
  rr_sign_start_ms = 0;
}

void setFault(const char* reason) {
  if (strcmp(fault_reason, "none") == 0) {
    strncpy(fault_reason, reason, sizeof(fault_reason) - 1);
    fault_reason[sizeof(fault_reason) - 1] = '\0';
  }
  controller_mode = MODE_FAULT;
}

void writeMotorOutputs(int left_us, int right_us) {
  int logical_left = clampPwm(left_us);
  int logical_right = clampPwm(right_us);
  int left_delta = logical_left - pwm_neutral_us;
  int right_delta = logical_right - pwm_neutral_us;
  int physical_left = pwm_neutral_us + left_motor_sign * left_delta;
  int physical_right = pwm_neutral_us + right_motor_sign * right_delta;
  current_left_pwm = logical_left;
  current_right_pwm = logical_right;
  motor_left.writeMicroseconds(clampPwm(physical_left));
  motor_right.writeMicroseconds(clampPwm(physical_right));
}

int slewToward(int current, int target, unsigned long dt_ms) {
  if (pwm_slew_us_per_s <= 0.0f || dt_ms == 0) {
    return target;
  }
  int max_delta = max(1, (int)roundf(pwm_slew_us_per_s * ((float)dt_ms / 1000.0f)));
  int delta = target - current;
  if (abs(delta) <= max_delta) {
    return target;
  }
  return current + (delta > 0 ? max_delta : -max_delta);
}

void applyNeutralNow() {
  target_left_pwm = pwm_neutral_us;
  target_right_pwm = pwm_neutral_us;
  current_left_pwm = pwm_neutral_us;
  current_right_pwm = pwm_neutral_us;
  motor_left.writeMicroseconds(pwm_neutral_us);
  motor_right.writeMicroseconds(pwm_neutral_us);
}

void resetPidState() {
  left_pid.Reset();
  right_pid.Reset();
  left_pid_output_us = 0.0f;
  right_pid_output_us = 0.0f;
  left_p_term = 0.0f;
  left_i_term = 0.0f;
  left_d_term = 0.0f;
  right_p_term = 0.0f;
  right_i_term = 0.0f;
  right_d_term = 0.0f;
}

void resetEncoderBaselines() {
  fl_ticks = readEncoderSigned(enc_fl, fl_encoder_sign);
  fr_ticks = readEncoderSigned(enc_fr, fr_encoder_sign);
  rl_ticks = readEncoderSigned(enc_rl, rl_encoder_sign);
  rr_ticks = readEncoderSigned(enc_rr, rr_encoder_sign);
  last_fl_ticks = fl_ticks;
  last_fr_ticks = fr_ticks;
  last_rl_ticks = rl_ticks;
  last_rr_ticks = rr_ticks;
  fl_tps = 0.0f;
  fr_tps = 0.0f;
  rl_tps = 0.0f;
  rr_tps = 0.0f;
  left_measured_tps = 0.0f;
  right_measured_tps = 0.0f;
}

void neutralizeForCriticalParam() {
  target_v_mps = 0.0f;
  target_omega_radps = 0.0f;
  left_target_tps = 0.0f;
  right_target_tps = 0.0f;
  left_error_tps = 0.0f;
  right_error_tps = 0.0f;
  resetPidState();
  applyNeutralNow();
  if (controller_mode != MODE_FAULT) {
    controller_mode = MODE_STOPPED;
  }
}

void configurePid() {
  left_pid.SetTunings(kp, ki, kd);
  right_pid.SetTunings(kp, ki, kd);
  left_pid.SetOutputLimits(-pid_output_limit_us, pid_output_limit_us);
  right_pid.SetOutputLimits(-pid_output_limit_us, pid_output_limit_us);
  left_pid.SetSampleTimeUs(control_interval_ms * 1000UL);
  right_pid.SetSampleTimeUs(control_interval_ms * 1000UL);
  left_pid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  right_pid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
  left_pid.SetMode(QuickPID::Control::automatic);
  right_pid.SetMode(QuickPID::Control::automatic);
}

void stopController(const char* reason, bool fault) {
  target_v_mps = 0.0f;
  target_omega_radps = 0.0f;
  left_target_tps = 0.0f;
  right_target_tps = 0.0f;
  left_error_tps = 0.0f;
  right_error_tps = 0.0f;
  resetPidState();
  applyNeutralNow();
  controller_mode = fault ? MODE_FAULT : MODE_STOPPED;
  if (fault) {
    setFault(reason);
  } else {
    clearFault();
  }
}

bool shouldFaultForWheel(
  const char* reason,
  float side_target_tps,
  float wheel_tps,
  float peer_tps,
  int side_pwm,
  unsigned long& start_ms
) {
  if (!stall_fault_enabled) {
    start_ms = 0;
    return false;
  }
  if (abs(side_target_tps) < stall_target_tps) {
    start_ms = 0;
    return false;
  }
  if (abs(side_pwm - pwm_neutral_us) < stall_pwm_delta_us) {
    start_ms = 0;
    return false;
  }
  if (abs(wheel_tps) > stall_near_zero_tps) {
    start_ms = 0;
    return false;
  }
  if (abs(peer_tps) < stall_moving_peer_tps) {
    start_ms = 0;
    return false;
  }
  if (start_ms == 0) {
    start_ms = millis();
    return false;
  }
  if (millis() - start_ms >= stall_timeout_ms) {
    setFault(reason);
    return true;
  }
  return false;
}

bool shouldFaultForSideStall(
  const char* reason,
  float side_target_tps,
  float first_tps,
  float second_tps,
  int side_pwm,
  unsigned long& start_ms
) {
  if (!stall_fault_enabled) {
    start_ms = 0;
    return false;
  }
  if (abs(side_target_tps) < stall_target_tps) {
    start_ms = 0;
    return false;
  }
  if (abs(side_pwm - pwm_neutral_us) < stall_pwm_delta_us) {
    start_ms = 0;
    return false;
  }
  if (abs(first_tps) > stall_near_zero_tps || abs(second_tps) > stall_near_zero_tps) {
    start_ms = 0;
    return false;
  }
  if (start_ms == 0) {
    start_ms = millis();
    return false;
  }
  if (millis() - start_ms >= stall_timeout_ms) {
    setFault(reason);
    return true;
  }
  return false;
}

bool shouldFaultForSideMismatch(
  const char* reason,
  float side_target_tps,
  float first_tps,
  float second_tps,
  int side_pwm,
  unsigned long& start_ms
) {
  if (!side_mismatch_fault_enabled) {
    start_ms = 0;
    return false;
  }
  if (abs(side_target_tps) < stall_target_tps) {
    start_ms = 0;
    return false;
  }
  if (abs(side_pwm - pwm_neutral_us) < stall_pwm_delta_us) {
    start_ms = 0;
    return false;
  }
  if (abs(first_tps - second_tps) < side_mismatch_fault_tps) {
    start_ms = 0;
    return false;
  }
  if (start_ms == 0) {
    start_ms = millis();
    return false;
  }
  if (millis() - start_ms >= stall_timeout_ms) {
    setFault(reason);
    return true;
  }
  return false;
}

bool shouldFaultForSignMismatch(
  const char* reason,
  float side_target_tps,
  float suspect_tps,
  float peer_tps,
  unsigned long& start_ms
) {
  int target_sign = signOf(side_target_tps, sign_mismatch_target_tps);
  if (target_sign == 0) {
    start_ms = 0;
    return false;
  }
  if (
    signOf(suspect_tps, sign_mismatch_tps) != -target_sign ||
    signOf(peer_tps, sign_mismatch_tps) != target_sign
  ) {
    start_ms = 0;
    return false;
  }
  if (start_ms == 0) {
    start_ms = millis();
    return false;
  }
  if (millis() - start_ms >= sign_mismatch_timeout_ms) {
    setFault(reason);
    return true;
  }
  return false;
}

bool checkEncoderDiagnostics() {
  if (controller_mode != MODE_VELOCITY) {
    fl_stall_start_ms = 0;
    rl_stall_start_ms = 0;
    fr_stall_start_ms = 0;
    rr_stall_start_ms = 0;
    left_side_stall_start_ms = 0;
    right_side_stall_start_ms = 0;
    left_mismatch_start_ms = 0;
    right_mismatch_start_ms = 0;
    fl_sign_start_ms = 0;
    rl_sign_start_ms = 0;
    fr_sign_start_ms = 0;
    rr_sign_start_ms = 0;
    return false;
  }

  if (shouldFaultForWheel("fl_stall", left_target_tps, fl_tps, rl_tps, current_left_pwm, fl_stall_start_ms)) return true;
  if (shouldFaultForWheel("rl_stall", left_target_tps, rl_tps, fl_tps, current_left_pwm, rl_stall_start_ms)) return true;
  if (shouldFaultForWheel("fr_stall", right_target_tps, fr_tps, rr_tps, current_right_pwm, fr_stall_start_ms)) return true;
  if (shouldFaultForWheel("rr_stall", right_target_tps, rr_tps, fr_tps, current_right_pwm, rr_stall_start_ms)) return true;
  if (shouldFaultForSideStall("left_side_stall", left_target_tps, fl_tps, rl_tps, current_left_pwm, left_side_stall_start_ms)) return true;
  if (shouldFaultForSideStall("right_side_stall", right_target_tps, fr_tps, rr_tps, current_right_pwm, right_side_stall_start_ms)) return true;
  if (shouldFaultForSideMismatch("left_mismatch", left_target_tps, fl_tps, rl_tps, current_left_pwm, left_mismatch_start_ms)) return true;
  if (shouldFaultForSideMismatch("right_mismatch", right_target_tps, fr_tps, rr_tps, current_right_pwm, right_mismatch_start_ms)) return true;

  if (shouldFaultForSignMismatch("fl_sign", left_target_tps, fl_tps, rl_tps, fl_sign_start_ms)) return true;
  if (shouldFaultForSignMismatch("rl_sign", left_target_tps, rl_tps, fl_tps, rl_sign_start_ms)) return true;
  if (shouldFaultForSignMismatch("fr_sign", right_target_tps, fr_tps, rr_tps, fr_sign_start_ms)) return true;
  if (shouldFaultForSignMismatch("rr_sign", right_target_tps, rr_tps, fr_tps, rr_sign_start_ms)) return true;

  return false;
}

void updateEncoderSpeeds(unsigned long dt_ms) {
  fl_ticks = readEncoderSigned(enc_fl, fl_encoder_sign);
  fr_ticks = readEncoderSigned(enc_fr, fr_encoder_sign);
  rl_ticks = readEncoderSigned(enc_rl, rl_encoder_sign);
  rr_ticks = readEncoderSigned(enc_rr, rr_encoder_sign);

  float dt_s = max(0.001f, (float)dt_ms / 1000.0f);
  fl_tps = (float)(fl_ticks - last_fl_ticks) / dt_s;
  fr_tps = (float)(fr_ticks - last_fr_ticks) / dt_s;
  rl_tps = (float)(rl_ticks - last_rl_ticks) / dt_s;
  rr_tps = (float)(rr_ticks - last_rr_ticks) / dt_s;

  if (
    encoder_jump_fault_enabled &&
    (controller_mode == MODE_VELOCITY || controller_mode == MODE_RAW2)
  ) {
    if (abs(fl_tps) > encoder_jump_tps) setFault("fl_jump");
    else if (abs(fr_tps) > encoder_jump_tps) setFault("fr_jump");
    else if (abs(rl_tps) > encoder_jump_tps) setFault("rl_jump");
    else if (abs(rr_tps) > encoder_jump_tps) setFault("rr_jump");
  }

  left_measured_tps = 0.5f * (fl_tps + rl_tps);
  right_measured_tps = 0.5f * (fr_tps + rr_tps);

  last_fl_ticks = fl_ticks;
  last_fr_ticks = fr_ticks;
  last_rl_ticks = rl_ticks;
  last_rr_ticks = rr_ticks;
}

void computeVelocityTargets() {
  float left_mps = target_v_mps - target_omega_radps * track_width_m * 0.5f;
  float right_mps = target_v_mps + target_omega_radps * track_width_m * 0.5f;
  float new_left_target_tps = mpsToTicksPerSec(left_mps);
  float new_right_target_tps = mpsToTicksPerSec(right_mps);

  if (abs(new_left_target_tps) < min_target_tps) {
    new_left_target_tps = 0.0f;
  }
  if (abs(new_right_target_tps) < min_target_tps) {
    new_right_target_tps = 0.0f;
  }

  if (
    signOf(left_target_tps, min_target_tps) != 0 &&
    signOf(new_left_target_tps, min_target_tps) != 0 &&
    signOf(left_target_tps, min_target_tps) != signOf(new_left_target_tps, min_target_tps)
  ) {
    resetPidState();
  }
  if (
    signOf(right_target_tps, min_target_tps) != 0 &&
    signOf(new_right_target_tps, min_target_tps) != 0 &&
    signOf(right_target_tps, min_target_tps) != signOf(new_right_target_tps, min_target_tps)
  ) {
    resetPidState();
  }

  left_target_tps = new_left_target_tps;
  right_target_tps = new_right_target_tps;
}

void updatePidAndOutputs(unsigned long dt_ms) {
  if (controller_mode == MODE_FAULT || strcmp(fault_reason, "none") != 0) {
    applyNeutralNow();
    return;
  }

  if (controller_mode == MODE_RAW2) {
    int left_next = slewToward(current_left_pwm, target_left_pwm, dt_ms);
    int right_next = slewToward(current_right_pwm, target_right_pwm, dt_ms);
    writeMotorOutputs(left_next, right_next);
    return;
  }

  if (controller_mode != MODE_VELOCITY) {
    applyNeutralNow();
    return;
  }

  computeVelocityTargets();
  left_error_tps = left_target_tps - left_measured_tps;
  right_error_tps = right_target_tps - right_measured_tps;

  if (left_target_tps == 0.0f && right_target_tps == 0.0f) {
    resetPidState();
    applyNeutralNow();
    return;
  }

  left_pid_input = left_measured_tps;
  right_pid_input = right_measured_tps;
  left_pid_setpoint = left_target_tps;
  right_pid_setpoint = right_target_tps;
  left_pid.Compute();
  right_pid.Compute();
  left_p_term = left_pid.GetPterm();
  left_i_term = left_pid.GetIterm();
  left_d_term = left_pid.GetDterm();
  right_p_term = right_pid.GetPterm();
  right_i_term = right_pid.GetIterm();
  right_d_term = right_pid.GetDterm();

  float left_delta_us = feedforward_us_per_tps * left_target_tps + left_pid_output_us;
  float right_delta_us = feedforward_us_per_tps * right_target_tps + right_pid_output_us;
  left_delta_us = clampFloat(left_delta_us, -(float)(pwm_max_us - pwm_neutral_us), (float)(pwm_max_us - pwm_neutral_us));
  right_delta_us = clampFloat(right_delta_us, -(float)(pwm_max_us - pwm_neutral_us), (float)(pwm_max_us - pwm_neutral_us));

  target_left_pwm = clampPwm((int)roundf((float)pwm_neutral_us + left_delta_us));
  target_right_pwm = clampPwm((int)roundf((float)pwm_neutral_us + right_delta_us));

  int left_next = slewToward(current_left_pwm, target_left_pwm, dt_ms);
  int right_next = slewToward(current_right_pwm, target_right_pwm, dt_ms);
  writeMotorOutputs(left_next, right_next);

  if (checkEncoderDiagnostics()) {
    stopController(fault_reason, true);
  }
}

void controlStep(unsigned long now_ms) {
  unsigned long dt_ms = now_ms - last_control_ms;
  if (dt_ms == 0) {
    return;
  }
  last_control_ms = now_ms;
  updateEncoderSpeeds(dt_ms);
  updatePidAndOutputs(dt_ms);
}

void printStatus(Stream& stream) {
  stream.print("S,");
  stream.print(millis()); stream.print(",");
  stream.print(fl_ticks); stream.print(",");
  stream.print(fr_ticks); stream.print(",");
  stream.print(rl_ticks); stream.print(",");
  stream.print(rr_ticks); stream.print(",");
  stream.print(fl_tps, 2); stream.print(",");
  stream.print(fr_tps, 2); stream.print(",");
  stream.print(rl_tps, 2); stream.print(",");
  stream.print(rr_tps, 2); stream.print(",");
  stream.print(left_target_tps, 2); stream.print(",");
  stream.print(right_target_tps, 2); stream.print(",");
  stream.print(left_measured_tps, 2); stream.print(",");
  stream.print(right_measured_tps, 2); stream.print(",");
  stream.print(current_left_pwm); stream.print(",");
  stream.print(current_right_pwm); stream.print(",");
  stream.print(left_error_tps, 2); stream.print(",");
  stream.print(right_error_tps, 2); stream.print(",");
  stream.print(left_p_term, 4); stream.print(",");
  stream.print(left_i_term, 4); stream.print(",");
  stream.print(left_d_term, 4); stream.print(",");
  stream.print(right_p_term, 4); stream.print(",");
  stream.print(right_i_term, 4); stream.print(",");
  stream.print(right_d_term, 4); stream.print(",");
  stream.println(fault_reason);

}

void sendStatus() {
  if (Serial) {
    printStatus(Serial);
  }
  printStatus(Serial1);
}

void printParamAck(Stream& stream, const char* name, bool ok) {
  stream.print("PARAM,");
  stream.print(name);
  stream.print(",");
  stream.println(ok ? "ok" : "unknown");
}

void sendParamAck(const char* name, bool ok) {
  if (Serial) {
    printParamAck(Serial, name, ok);
  }
  printParamAck(Serial1, name, ok);
}

void printControlAck(Stream& stream, const char* name, const char* value) {
  stream.print("CTRL,");
  stream.print(name);
  stream.print(",");
  stream.println(value);
}

void sendControlAck(const char* name, const char* value) {
  if (Serial) {
    printControlAck(Serial, name, value);
  }
  printControlAck(Serial1, name, value);
}

void printParamDump(Stream& stream) {
  stream.print("PARAMS,track_width_m=");
  stream.print(track_width_m, 6);
  stream.print(",wheel_radius_m=");
  stream.print(wheel_radius_m, 6);
  stream.print(",ticks_per_rev=");
  stream.print(ticks_per_rev, 2);
  stream.print(",kp=");
  stream.print(kp, 6);
  stream.print(",ki=");
  stream.print(ki, 6);
  stream.print(",kd=");
  stream.print(kd, 6);
  stream.print(",pid_backend=QuickPID");
  stream.print(",left_motor_sign=");
  stream.print(left_motor_sign);
  stream.print(",right_motor_sign=");
  stream.print(right_motor_sign);
  stream.print(",fl_encoder_sign=");
  stream.print(fl_encoder_sign);
  stream.print(",fr_encoder_sign=");
  stream.print(fr_encoder_sign);
  stream.print(",rl_encoder_sign=");
  stream.print(rl_encoder_sign);
  stream.print(",rr_encoder_sign=");
  stream.print(rr_encoder_sign);
  stream.print(",pwm_min_us=");
  stream.print(pwm_min_us);
  stream.print(",pwm_neutral_us=");
  stream.print(pwm_neutral_us);
  stream.print(",pwm_max_us=");
  stream.print(pwm_max_us);
  stream.print(",control_hz=");
  stream.print(1000.0f / max(1.0f, (float)control_interval_ms), 2);
  stream.print(",side_mismatch_warn_tps=");
  stream.print(side_mismatch_warn_tps, 2);
  stream.print(",side_mismatch_fault_tps=");
  stream.print(side_mismatch_fault_tps, 2);
  stream.print(",sign_mismatch_tps=");
  stream.print(sign_mismatch_tps, 2);
  stream.print(",sign_mismatch_target_tps=");
  stream.print(sign_mismatch_target_tps, 2);
  stream.print(",sign_mismatch_timeout_ms=");
  stream.print(sign_mismatch_timeout_ms);
  stream.print(",encoder_jump_tps=");
  stream.print(encoder_jump_tps, 2);
  stream.print(",command_timeout_ms=");
  stream.print(command_timeout_ms, 0);
  stream.print(",status_stream_enabled=");
  stream.println(status_stream_enabled ? 1 : 0);
}

void sendParamDump() {
  if (Serial) {
    printParamDump(Serial);
  }
  printParamDump(Serial1);
}

bool setParam(const char* name, float value) {
  bool critical = false;
  bool neutralize_outputs = false;
  bool reset_encoder_baselines = false;

  if (strcmp(name, "kp") == 0) { kp = value; critical = true; }
  else if (strcmp(name, "ki") == 0) { ki = value; critical = true; }
  else if (strcmp(name, "kd") == 0) { kd = value; critical = true; }
  else if (strcmp(name, "ff_us_per_tps") == 0) { feedforward_us_per_tps = value; critical = true; }
  else if (strcmp(name, "pid_output_limit_us") == 0) { pid_output_limit_us = max(1.0f, value); critical = true; }
  else if (strcmp(name, "pwm_slew_us_per_s") == 0) { pwm_slew_us_per_s = max(0.0f, value); critical = true; }
  else if (strcmp(name, "control_hz") == 0) { control_interval_ms = (unsigned long)clampFloat(roundf(1000.0f / max(1.0f, value)), 5.0f, 50.0f); critical = true; }
  else if (strcmp(name, "control_interval_ms") == 0) { control_interval_ms = (unsigned long)clampFloat(roundf(value), 5.0f, 50.0f); critical = true; }
  else if (strcmp(name, "track_width_m") == 0) { track_width_m = max(0.01f, value); critical = true; }
  else if (strcmp(name, "wheel_radius_m") == 0) { wheel_radius_m = max(0.001f, value); critical = true; }
  else if (strcmp(name, "ticks_per_rev") == 0) { ticks_per_rev = max(1.0f, value); critical = true; }
  else if (strcmp(name, "command_timeout_ms") == 0) command_timeout_ms = max(50.0f, value);
  else if (strcmp(name, "min_target_tps") == 0) { min_target_tps = max(0.0f, value); critical = true; }
  else if (strcmp(name, "deadband_tps") == 0) { deadband_tps = max(0.0f, value); critical = true; }
  else if (strcmp(name, "pwm_min_us") == 0) { pwm_min_us = (int)value; critical = true; neutralize_outputs = true; }
  else if (strcmp(name, "pwm_neutral_us") == 0) { pwm_neutral_us = (int)value; critical = true; neutralize_outputs = true; }
  else if (strcmp(name, "pwm_max_us") == 0) { pwm_max_us = (int)value; critical = true; neutralize_outputs = true; }
  else if (strcmp(name, "fl_encoder_sign") == 0) { fl_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "fr_encoder_sign") == 0) { fr_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "rl_encoder_sign") == 0) { rl_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "rr_encoder_sign") == 0) { rr_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "left_motor_sign") == 0) { left_motor_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; }
  else if (strcmp(name, "right_motor_sign") == 0) { right_motor_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; }
  else if (strcmp(name, "stall_fault_enabled") == 0) stall_fault_enabled = value != 0.0f;
  else if (strcmp(name, "stall_target_tps") == 0) stall_target_tps = max(0.0f, value);
  else if (strcmp(name, "stall_near_zero_tps") == 0) stall_near_zero_tps = max(0.0f, value);
  else if (strcmp(name, "stall_moving_peer_tps") == 0) stall_moving_peer_tps = max(0.0f, value);
  else if (strcmp(name, "stall_pwm_delta_us") == 0) stall_pwm_delta_us = max(0.0f, value);
  else if (strcmp(name, "stall_timeout_ms") == 0) stall_timeout_ms = (unsigned long)max(0.0f, value);
  else if (strcmp(name, "sign_mismatch_tps") == 0) sign_mismatch_tps = max(0.0f, value);
  else if (strcmp(name, "sign_mismatch_target_tps") == 0) sign_mismatch_target_tps = max(0.0f, value);
  else if (strcmp(name, "sign_mismatch_timeout_ms") == 0) sign_mismatch_timeout_ms = (unsigned long)max(0.0f, value);
  else if (strcmp(name, "side_mismatch_fault_enabled") == 0) side_mismatch_fault_enabled = value != 0.0f;
  else if (strcmp(name, "side_mismatch_warn_tps") == 0) side_mismatch_warn_tps = max(0.0f, value);
  else if (strcmp(name, "side_mismatch_fault_tps") == 0) side_mismatch_fault_tps = max(0.0f, value);
  else if (strcmp(name, "encoder_jump_fault_enabled") == 0) encoder_jump_fault_enabled = value != 0.0f;
  else if (strcmp(name, "encoder_jump_tps") == 0) encoder_jump_tps = max(0.0f, value);
  else return false;

  configurePid();
  if (reset_encoder_baselines) {
    resetEncoderBaselines();
  }
  if (neutralize_outputs) {
    neutralizeForCriticalParam();
  } else if (critical) {
    resetPidState();
  }
  return true;
}

void parseCommand(char* s, const char* transport_name) {
  char* token = strtok(s, " ");
  if (token == NULL) {
    return;
  }
  if (strcmp(token, "CMD") != 0) {
    return;
  }

  char* verb = strtok(NULL, " ");
  if (verb == NULL) {
    return;
  }

  if (strcmp(verb, "STOP") == 0) {
    last_command_ms = millis();
    stopController("none", false);
    return;
  }

  if (strcmp(verb, "STATUS") == 0) {
    sendStatus();
    sendParamDump();
    return;
  }

  if (strcmp(verb, "STATUS_STREAM") == 0) {
    char* value_text = strtok(NULL, " ");
    if (value_text == NULL) {
      return;
    }
    status_stream_enabled = atoi(value_text) != 0;
    sendControlAck("status_stream", status_stream_enabled ? "on" : "off");
    return;
  }

  if (strcmp(verb, "PARAMDUMP") == 0) {
    sendParamDump();
    return;
  }

  if (strcmp(verb, "V") == 0) {
    char* v_text = strtok(NULL, " ");
    char* omega_text = strtok(NULL, " ");
    if (v_text == NULL || omega_text == NULL) {
      return;
    }
    last_command_ms = millis();
    if (controller_mode == MODE_FAULT || strcmp(fault_reason, "none") != 0) {
      return;
    }
    target_v_mps = atof(v_text);
    target_omega_radps = atof(omega_text);
    clearFault();
    controller_mode = MODE_VELOCITY;
    return;
  }

  if (strcmp(verb, "RAW2") == 0) {
    char* left_text = strtok(NULL, " ");
    char* right_text = strtok(NULL, " ");
    if (left_text == NULL || right_text == NULL) {
      return;
    }
    last_command_ms = millis();
    if (controller_mode == MODE_FAULT || strcmp(fault_reason, "none") != 0) {
      return;
    }
    target_left_pwm = clampPwm(atoi(left_text));
    target_right_pwm = clampPwm(atoi(right_text));
    clearFault();
    resetPidState();
    controller_mode = MODE_RAW2;
    return;
  }

  if (strcmp(verb, "PARAM") == 0) {
    char* name = strtok(NULL, " ");
    char* value_text = strtok(NULL, " ");
    if (name == NULL || value_text == NULL) {
      return;
    }
    bool ok = setParam(name, atof(value_text));
    sendParamAck(name, ok);
    if (Serial) {
      Serial.print("DBG PARAM ");
      Serial.print(transport_name);
      Serial.print(" ");
      Serial.print(name);
      Serial.println(ok ? " ok" : " unknown");
    }
  }
}

void processTransport(Stream& stream, char* buf, int& buf_idx, const char* transport_name) {
  while (stream.available()) {
    char c = stream.read();
    if (c == '\n') {
      buf[buf_idx] = '\0';
      parseCommand(buf, transport_name);
      buf_idx = 0;
    } else if (c != '\r' && buf_idx < 95) {
      buf[buf_idx++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  motor_left.attach(PWM_L, 1000, 2000);
  motor_right.attach(PWM_R, 1000, 2000);
  applyNeutralNow();

  last_control_ms = millis();
  last_status_ms = millis();
  last_command_ms = millis();
  fl_ticks = readEncoderSigned(enc_fl, fl_encoder_sign);
  fr_ticks = readEncoderSigned(enc_fr, fr_encoder_sign);
  rl_ticks = readEncoderSigned(enc_rl, rl_encoder_sign);
  rr_ticks = readEncoderSigned(enc_rr, rr_encoder_sign);
  last_fl_ticks = fl_ticks;
  last_fr_ticks = fr_ticks;
  last_rl_ticks = rl_ticks;
  last_rr_ticks = rr_ticks;
  clearFault();
  configurePid();

  if (Serial) {
    Serial.println("Teensy 4.1 side PID controller ready");
    Serial.println("DBG PID backend QuickPID");
  }
}

void loop() {
  if (Serial) {
    processTransport(Serial, usb_buf, usb_buf_idx, "usb");
  }
  processTransport(Serial1, uart_buf, uart_buf_idx, "uart");

  unsigned long now_ms = millis();
  if (
    controller_mode != MODE_STOPPED &&
    controller_mode != MODE_FAULT &&
    now_ms - last_command_ms > (unsigned long)command_timeout_ms
  ) {
    stopController("none", false);
  }

  if (now_ms - last_control_ms >= control_interval_ms) {
    controlStep(now_ms);
  }

  if (status_stream_enabled && now_ms - last_status_ms >= STATUS_INTERVAL_MS) {
    last_status_ms = now_ms;
    sendStatus();
  }
}
