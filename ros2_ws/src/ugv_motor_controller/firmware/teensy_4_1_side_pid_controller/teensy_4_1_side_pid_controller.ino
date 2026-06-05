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
#include <math.h>
#include <QuickPID.h>
#include <Servo.h>

#define ENC_REAR_LEFT_A    27
#define ENC_REAR_LEFT_B    26
#define ENC_REAR_RIGHT_A   31
#define ENC_REAR_RIGHT_B   30
#define ENC_FRONT_LEFT_A   25
#define ENC_FRONT_LEFT_B   24
#define ENC_FRONT_RIGHT_A  29
#define ENC_FRONT_RIGHT_B  28
#define PWM_L              2
#define PWM_R              3

const unsigned long DEFAULT_CONTROL_INTERVAL_MS = 20;  // 50 Hz
const unsigned long STATUS_INTERVAL_MS = 50;   // 20 Hz
const unsigned long DEFAULT_COMMAND_TIMEOUT_MS = 500;
const unsigned long HEARTBEAT_LED_INTERVAL_MS = 500;
const char FIRMWARE_ID[] = "teensy_4_1_side_pid_v2026_06_04";

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

const int HEARTBEAT_LED_PIN = LED_BUILTIN;

const int COMMAND_BUFFER_LEN = 96;
const int DEFAULT_PWM_MIN_US = 1100;
const int DEFAULT_PWM_NEUTRAL_US = 1500;
const int DEFAULT_PWM_MAX_US = 1900;
const int HARD_PWM_MIN_US = 1000;
const int HARD_PWM_MAX_US = 2000;
const int MIN_PWM_SPAN_US = 20;

const float DEFAULT_TRACK_WIDTH_M = 0.416f;
const float DEFAULT_WHEEL_RADIUS_M = 0.0825f;
const float DEFAULT_TICKS_PER_REV = 3200.0f;
const float DEFAULT_KP = 0.03f;
const float DEFAULT_KI = 0.0f;
const float DEFAULT_KD = 0.0f;
const float DEFAULT_FF_US_PER_TPS = 0.02f;
const float DEFAULT_STATIC_FF_US = 90.0f;
const float DEFAULT_STATIC_FF_FULL_TARGET_TPS = 1500.0f;
const float DEFAULT_PID_OUTPUT_LIMIT_US = 180.0f;

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
unsigned long last_heartbeat_led_ms = 0;
bool heartbeat_led_on = false;
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
unsigned long right_reverse_unavailable_start_ms = 0;

char usb_buf[COMMAND_BUFFER_LEN];
char uart_buf[COMMAND_BUFFER_LEN];
int usb_buf_idx = 0;
int uart_buf_idx = 0;
bool usb_buf_overflow = false;
bool uart_buf_overflow = false;

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
float feedforward_us_per_tps = DEFAULT_FF_US_PER_TPS;
float left_feedforward_us_per_tps = DEFAULT_FF_US_PER_TPS;
float right_feedforward_us_per_tps = DEFAULT_FF_US_PER_TPS;
float right_reverse_feedforward_us_per_tps = -1.0f;
float static_ff_us = DEFAULT_STATIC_FF_US;
float left_static_ff_us = DEFAULT_STATIC_FF_US;
float right_static_ff_us = DEFAULT_STATIC_FF_US;
float right_reverse_static_ff_us = -1.0f;
float right_reverse_pwm_floor_us = 0.0f;
float static_ff_full_target_tps = DEFAULT_STATIC_FF_FULL_TARGET_TPS;
float pid_output_limit_us = DEFAULT_PID_OUTPUT_LIMIT_US;
float left_pid_output_limit_us = DEFAULT_PID_OUTPUT_LIMIT_US;
float right_pid_output_limit_us = DEFAULT_PID_OUTPUT_LIMIT_US;
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
float right_reverse_unavailable_target_tps = 1000.0f;
float right_reverse_unavailable_near_zero_tps = 250.0f;
unsigned long right_reverse_unavailable_timeout_ms = 400;

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

bool validPwmConfig(int min_us, int neutral_us, int max_us) {
  if (min_us < HARD_PWM_MIN_US || max_us > HARD_PWM_MAX_US) {
    return false;
  }
  if (min_us >= neutral_us || neutral_us >= max_us) {
    return false;
  }
  if (neutral_us - min_us < MIN_PWM_SPAN_US || max_us - neutral_us < MIN_PWM_SPAN_US) {
    return false;
  }
  return true;
}

bool parseFiniteFloatToken(const char* text, float& out) {
  if (text == NULL || text[0] == '\0') {
    return false;
  }
  char* end = NULL;
  float value = strtof(text, &end);
  if (end == text || end == NULL || *end != '\0' || !isfinite(value)) {
    return false;
  }
  out = value;
  return true;
}

bool parseIntToken(const char* text, int& out) {
  float value = 0.0f;
  if (!parseFiniteFloatToken(text, value)) {
    return false;
  }
  if (value < -32768.0f || value > 32767.0f) {
    return false;
  }
  out = (int)roundf(value);
  return true;
}

bool commandHasExtraTokens() {
  return strtok(NULL, " ") != NULL;
}

bool valueInRange(float value, float lo, float hi) {
  return isfinite(value) && value >= lo && value <= hi;
}

bool assignFloatInRange(float& target, float value, float lo, float hi) {
  if (!valueInRange(value, lo, hi)) {
    return false;
  }
  target = value;
  return true;
}

bool assignUnsignedLongInRange(unsigned long& target, float value, float lo, float hi) {
  if (!valueInRange(value, lo, hi)) {
    return false;
  }
  target = (unsigned long)roundf(value);
  return true;
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

float staticFeedforwardForTarget(float target_tps, float side_static_ff_us) {
  int target_sign = signOf(target_tps, min_target_tps);
  if (target_sign == 0) {
    return 0.0f;
  }
  float full_target = max(min_target_tps + 1.0f, static_ff_full_target_tps);
  float target_abs = fabsf(target_tps);
  float ramp = clampFloat((target_abs - min_target_tps) / (full_target - min_target_tps), 0.0f, 1.0f);
  return (float)target_sign * max(0.0f, side_static_ff_us) * ramp;
}

float feedforwardForTarget(float target_tps, float side_static_ff_us, float side_ff_us_per_tps) {
  if (signOf(target_tps, min_target_tps) == 0) {
    return 0.0f;
  }
  return staticFeedforwardForTarget(target_tps, side_static_ff_us) + side_ff_us_per_tps * target_tps;
}

float rightStaticFfForTarget(float target_tps) {
  if (target_tps < -min_target_tps && right_reverse_static_ff_us >= 0.0f) {
    return right_reverse_static_ff_us;
  }
  return right_static_ff_us;
}

float rightFeedforwardForTarget(float target_tps) {
  if (target_tps < -min_target_tps && right_reverse_feedforward_us_per_tps >= 0.0f) {
    return right_reverse_feedforward_us_per_tps;
  }
  return right_feedforward_us_per_tps;
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
  right_reverse_unavailable_start_ms = 0;
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

void resetLeftPidState() {
  left_pid.Reset();
  left_pid_output_us = 0.0f;
  left_p_term = 0.0f;
  left_i_term = 0.0f;
  left_d_term = 0.0f;
}

void resetRightPidState() {
  right_pid.Reset();
  right_pid_output_us = 0.0f;
  right_p_term = 0.0f;
  right_i_term = 0.0f;
  right_d_term = 0.0f;
}

void setPidOutputLimitsForBase(QuickPID& pid, float base_delta_us, float side_output_limit_us) {
  float positive_range_us = max(1.0f, (float)(pwm_max_us - pwm_neutral_us));
  float negative_range_us = max(1.0f, (float)(pwm_neutral_us - pwm_min_us));
  float limit_us = max(1.0f, side_output_limit_us);
  float lo = max(-limit_us, -negative_range_us - base_delta_us);
  float hi = min(limit_us, positive_range_us - base_delta_us);
  if (lo > hi) {
    float forced = clampFloat(-base_delta_us, -limit_us, limit_us);
    lo = forced - 0.001f;
    hi = forced + 0.001f;
  }
  pid.SetOutputLimits(lo, hi);
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
  left_pid.SetOutputLimits(-left_pid_output_limit_us, left_pid_output_limit_us);
  right_pid.SetOutputLimits(-right_pid_output_limit_us, right_pid_output_limit_us);
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
  if (fabsf(side_target_tps) < stall_target_tps) {
    start_ms = 0;
    return false;
  }
  if (fabsf((float)(side_pwm - pwm_neutral_us)) < stall_pwm_delta_us) {
    start_ms = 0;
    return false;
  }
  if (fabsf(wheel_tps) > stall_near_zero_tps) {
    start_ms = 0;
    return false;
  }
  if (fabsf(peer_tps) < stall_moving_peer_tps) {
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
  if (fabsf(side_target_tps) < stall_target_tps) {
    start_ms = 0;
    return false;
  }
  if (fabsf((float)(side_pwm - pwm_neutral_us)) < stall_pwm_delta_us) {
    start_ms = 0;
    return false;
  }
  if (fabsf(first_tps) > stall_near_zero_tps || fabsf(second_tps) > stall_near_zero_tps) {
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
  if (fabsf(side_target_tps) < stall_target_tps) {
    start_ms = 0;
    return false;
  }
  if (fabsf((float)(side_pwm - pwm_neutral_us)) < stall_pwm_delta_us) {
    start_ms = 0;
    return false;
  }
  if (fabsf(first_tps - second_tps) < side_mismatch_fault_tps) {
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

bool shouldFaultForRightReverseUnavailable() {
  if (!stall_fault_enabled) {
    right_reverse_unavailable_start_ms = 0;
    return false;
  }
  if (right_target_tps > -right_reverse_unavailable_target_tps) {
    right_reverse_unavailable_start_ms = 0;
    return false;
  }
  if ((float)(pwm_neutral_us - current_right_pwm) < stall_pwm_delta_us) {
    right_reverse_unavailable_start_ms = 0;
    return false;
  }
  if (fabsf(right_measured_tps) > right_reverse_unavailable_near_zero_tps) {
    right_reverse_unavailable_start_ms = 0;
    return false;
  }
  if (right_reverse_unavailable_start_ms == 0) {
    right_reverse_unavailable_start_ms = millis();
    return false;
  }
  if (millis() - right_reverse_unavailable_start_ms >= right_reverse_unavailable_timeout_ms) {
    setFault("right_reverse_unavailable");
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
    right_reverse_unavailable_start_ms = 0;
    return false;
  }

  if (shouldFaultForRightReverseUnavailable()) return true;
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
    if (fabsf(fl_tps) > encoder_jump_tps) setFault("fl_jump");
    else if (fabsf(fr_tps) > encoder_jump_tps) setFault("fr_jump");
    else if (fabsf(rl_tps) > encoder_jump_tps) setFault("rl_jump");
    else if (fabsf(rr_tps) > encoder_jump_tps) setFault("rr_jump");
  }

  left_measured_tps = 0.5f * (fl_tps + rl_tps);
  right_measured_tps = 0.5f * (fr_tps + rr_tps);

  last_fl_ticks = fl_ticks;
  last_fr_ticks = fr_ticks;
  last_rl_ticks = rl_ticks;
  last_rr_ticks = rr_ticks;
}

void computeVelocityTargets() {
  if (
    !isfinite(target_v_mps) ||
    !isfinite(target_omega_radps) ||
    !isfinite(track_width_m) ||
    !isfinite(wheel_radius_m) ||
    !isfinite(ticks_per_rev)
  ) {
    setFault("nonfinite_state");
    left_target_tps = 0.0f;
    right_target_tps = 0.0f;
    return;
  }

  float left_mps = target_v_mps - target_omega_radps * track_width_m * 0.5f;
  float right_mps = target_v_mps + target_omega_radps * track_width_m * 0.5f;
  float new_left_target_tps = mpsToTicksPerSec(left_mps);
  float new_right_target_tps = mpsToTicksPerSec(right_mps);

  if (!isfinite(new_left_target_tps) || !isfinite(new_right_target_tps)) {
    setFault("nonfinite_target");
    left_target_tps = 0.0f;
    right_target_tps = 0.0f;
    return;
  }

  if (fabsf(new_left_target_tps) < min_target_tps) {
    new_left_target_tps = 0.0f;
  }
  if (fabsf(new_right_target_tps) < min_target_tps) {
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

  bool left_active = fabsf(left_target_tps) >= min_target_tps;
  bool right_active = fabsf(right_target_tps) >= min_target_tps;
  if (!left_active && !right_active) {
    resetPidState();
    applyNeutralNow();
    return;
  }

  float left_base_delta_us = feedforwardForTarget(
    left_target_tps,
    left_static_ff_us,
    left_feedforward_us_per_tps
  );
  float right_base_delta_us = feedforwardForTarget(
    right_target_tps,
    rightStaticFfForTarget(right_target_tps),
    rightFeedforwardForTarget(right_target_tps)
  );

  if (left_active) {
    setPidOutputLimitsForBase(left_pid, left_base_delta_us, left_pid_output_limit_us);
    left_pid_input = left_measured_tps;
    left_pid_setpoint = left_target_tps;
    left_pid.Compute();
    left_p_term = left_pid.GetPterm();
    left_i_term = left_pid.GetIterm();
    left_d_term = left_pid.GetDterm();
  } else {
    resetLeftPidState();
  }

  if (right_active) {
    setPidOutputLimitsForBase(right_pid, right_base_delta_us, right_pid_output_limit_us);
    right_pid_input = right_measured_tps;
    right_pid_setpoint = right_target_tps;
    right_pid.Compute();
    right_p_term = right_pid.GetPterm();
    right_i_term = right_pid.GetIterm();
    right_d_term = right_pid.GetDterm();
  } else {
    resetRightPidState();
  }

  float left_delta_us = left_active ? left_base_delta_us + left_pid_output_us : 0.0f;
  float right_delta_us = right_active ? right_base_delta_us + right_pid_output_us : 0.0f;
  if (!isfinite(left_delta_us) || !isfinite(right_delta_us)) {
    stopController("nonfinite_pid", true);
    return;
  }
  float positive_pwm_range_us = (float)(pwm_max_us - pwm_neutral_us);
  float negative_pwm_range_us = (float)(pwm_neutral_us - pwm_min_us);
  if (right_active && right_target_tps < -min_target_tps && right_reverse_pwm_floor_us > 0.0f) {
    right_delta_us = min(right_delta_us, -min(right_reverse_pwm_floor_us, negative_pwm_range_us));
  }
  left_delta_us = clampFloat(left_delta_us, -negative_pwm_range_us, positive_pwm_range_us);
  right_delta_us = clampFloat(right_delta_us, -negative_pwm_range_us, positive_pwm_range_us);

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
  stream.print("PARAMS,firmware_id=");
  stream.print(FIRMWARE_ID);
  stream.print(",track_width_m=");
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
  stream.print(",ff_us_per_tps=");
  stream.print(feedforward_us_per_tps, 6);
  stream.print(",left_ff_us_per_tps=");
  stream.print(left_feedforward_us_per_tps, 6);
  stream.print(",right_ff_us_per_tps=");
  stream.print(right_feedforward_us_per_tps, 6);
  stream.print(",right_reverse_ff_us_per_tps=");
  stream.print(right_reverse_feedforward_us_per_tps, 6);
  stream.print(",static_ff_us=");
  stream.print(static_ff_us, 2);
  stream.print(",static_ff_full_target_tps=");
  stream.print(static_ff_full_target_tps, 2);
  stream.print(",left_static_ff_us=");
  stream.print(left_static_ff_us, 2);
  stream.print(",right_static_ff_us=");
  stream.print(right_static_ff_us, 2);
  stream.print(",right_reverse_static_ff_us=");
  stream.print(right_reverse_static_ff_us, 2);
  stream.print(",right_reverse_pwm_floor_us=");
  stream.print(right_reverse_pwm_floor_us, 2);
  stream.print(",right_reverse_unavailable_target_tps=");
  stream.print(right_reverse_unavailable_target_tps, 2);
  stream.print(",right_reverse_unavailable_near_zero_tps=");
  stream.print(right_reverse_unavailable_near_zero_tps, 2);
  stream.print(",right_reverse_unavailable_timeout_ms=");
  stream.print(right_reverse_unavailable_timeout_ms);
  stream.print(",pid_output_limit_us=");
  stream.print(pid_output_limit_us, 2);
  stream.print(",left_pid_output_limit_us=");
  stream.print(left_pid_output_limit_us, 2);
  stream.print(",right_pid_output_limit_us=");
  stream.print(right_pid_output_limit_us, 2);
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

  if (strcmp(name, "kp") == 0) {
    if (!assignFloatInRange(kp, value, 0.0f, 5.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "ki") == 0) {
    if (!assignFloatInRange(ki, value, 0.0f, 5.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "kd") == 0) {
    if (!assignFloatInRange(kd, value, 0.0f, 5.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "ff_us_per_tps") == 0) {
    if (!assignFloatInRange(feedforward_us_per_tps, value, 0.0f, 2.0f)) return false;
    left_feedforward_us_per_tps = feedforward_us_per_tps;
    right_feedforward_us_per_tps = feedforward_us_per_tps;
    critical = true;
  }
  else if (strcmp(name, "left_ff_us_per_tps") == 0) {
    if (!assignFloatInRange(left_feedforward_us_per_tps, value, 0.0f, 2.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "right_ff_us_per_tps") == 0) {
    if (!assignFloatInRange(right_feedforward_us_per_tps, value, 0.0f, 2.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "right_reverse_ff_us_per_tps") == 0) {
    if (!assignFloatInRange(right_reverse_feedforward_us_per_tps, value, -1.0f, 2.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "static_ff_us") == 0) {
    if (!assignFloatInRange(static_ff_us, value, 0.0f, 500.0f)) return false;
    left_static_ff_us = static_ff_us;
    right_static_ff_us = static_ff_us;
    critical = true;
  }
  else if (strcmp(name, "static_feedforward_us") == 0) {
    if (!assignFloatInRange(static_ff_us, value, 0.0f, 500.0f)) return false;
    left_static_ff_us = static_ff_us;
    right_static_ff_us = static_ff_us;
    critical = true;
  }
  else if (strcmp(name, "static_ff_full_target_tps") == 0) {
    if (!assignFloatInRange(static_ff_full_target_tps, value, 1.0f, 50000.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "left_static_ff_us") == 0) {
    if (!assignFloatInRange(left_static_ff_us, value, 0.0f, 500.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "right_static_ff_us") == 0) {
    if (!assignFloatInRange(right_static_ff_us, value, 0.0f, 500.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "right_reverse_static_ff_us") == 0) {
    if (!assignFloatInRange(right_reverse_static_ff_us, value, -1.0f, 500.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "right_reverse_pwm_floor_us") == 0) {
    if (!assignFloatInRange(right_reverse_pwm_floor_us, value, 0.0f, 500.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "right_reverse_unavailable_target_tps") == 0) {
    if (!assignFloatInRange(right_reverse_unavailable_target_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "right_reverse_unavailable_near_zero_tps") == 0) {
    if (!assignFloatInRange(right_reverse_unavailable_near_zero_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "right_reverse_unavailable_timeout_ms") == 0) {
    if (!assignUnsignedLongInRange(right_reverse_unavailable_timeout_ms, value, 0.0f, 10000.0f)) return false;
  }
  else if (strcmp(name, "pid_output_limit_us") == 0) {
    if (!assignFloatInRange(pid_output_limit_us, value, 1.0f, 500.0f)) return false;
    left_pid_output_limit_us = pid_output_limit_us;
    right_pid_output_limit_us = pid_output_limit_us;
    critical = true;
  }
  else if (strcmp(name, "left_pid_output_limit_us") == 0) {
    if (!assignFloatInRange(left_pid_output_limit_us, value, 1.0f, 500.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "right_pid_output_limit_us") == 0) {
    if (!assignFloatInRange(right_pid_output_limit_us, value, 1.0f, 500.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "pwm_slew_us_per_s") == 0) {
    if (!assignFloatInRange(pwm_slew_us_per_s, value, 0.0f, 10000.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "control_hz") == 0) { control_interval_ms = (unsigned long)clampFloat(roundf(1000.0f / max(1.0f, value)), 5.0f, 50.0f); critical = true; }
  else if (strcmp(name, "control_interval_ms") == 0) { control_interval_ms = (unsigned long)clampFloat(roundf(value), 5.0f, 50.0f); critical = true; }
  else if (strcmp(name, "track_width_m") == 0) {
    if (!assignFloatInRange(track_width_m, value, 0.05f, 2.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "wheel_radius_m") == 0) {
    if (!assignFloatInRange(wheel_radius_m, value, 0.02f, 0.30f)) return false;
    critical = true;
  }
  else if (strcmp(name, "ticks_per_rev") == 0) {
    if (!assignFloatInRange(ticks_per_rev, value, 1.0f, 100000.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "command_timeout_ms") == 0) {
    if (!assignFloatInRange(command_timeout_ms, value, 50.0f, 5000.0f)) return false;
  }
  else if (strcmp(name, "min_target_tps") == 0) {
    if (!assignFloatInRange(min_target_tps, value, 0.0f, 1000.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "deadband_tps") == 0) {
    if (!assignFloatInRange(deadband_tps, value, 0.0f, 1000.0f)) return false;
    critical = true;
  }
  else if (strcmp(name, "pwm_min_us") == 0) {
    int candidate = (int)roundf(value);
    if (!validPwmConfig(candidate, pwm_neutral_us, pwm_max_us)) return false;
    pwm_min_us = candidate;
    critical = true;
    neutralize_outputs = true;
  }
  else if (strcmp(name, "pwm_neutral_us") == 0) {
    int candidate = (int)roundf(value);
    if (!validPwmConfig(pwm_min_us, candidate, pwm_max_us)) return false;
    pwm_neutral_us = candidate;
    critical = true;
    neutralize_outputs = true;
  }
  else if (strcmp(name, "pwm_max_us") == 0) {
    int candidate = (int)roundf(value);
    if (!validPwmConfig(pwm_min_us, pwm_neutral_us, candidate)) return false;
    pwm_max_us = candidate;
    critical = true;
    neutralize_outputs = true;
  }
  else if (strcmp(name, "fl_encoder_sign") == 0) { fl_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "fr_encoder_sign") == 0) { fr_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "rl_encoder_sign") == 0) { rl_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "rr_encoder_sign") == 0) { rr_encoder_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; reset_encoder_baselines = true; }
  else if (strcmp(name, "left_motor_sign") == 0) { left_motor_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; }
  else if (strcmp(name, "right_motor_sign") == 0) { right_motor_sign = value < 0 ? -1 : 1; critical = true; neutralize_outputs = true; }
  else if (strcmp(name, "stall_fault_enabled") == 0) stall_fault_enabled = value != 0.0f;
  else if (strcmp(name, "stall_target_tps") == 0) {
    if (!assignFloatInRange(stall_target_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "stall_near_zero_tps") == 0) {
    if (!assignFloatInRange(stall_near_zero_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "stall_moving_peer_tps") == 0) {
    if (!assignFloatInRange(stall_moving_peer_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "stall_pwm_delta_us") == 0) {
    if (!assignFloatInRange(stall_pwm_delta_us, value, 0.0f, 500.0f)) return false;
  }
  else if (strcmp(name, "stall_timeout_ms") == 0) {
    if (!assignUnsignedLongInRange(stall_timeout_ms, value, 0.0f, 10000.0f)) return false;
  }
  else if (strcmp(name, "sign_mismatch_tps") == 0) {
    if (!assignFloatInRange(sign_mismatch_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "sign_mismatch_target_tps") == 0) {
    if (!assignFloatInRange(sign_mismatch_target_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "sign_mismatch_timeout_ms") == 0) {
    if (!assignUnsignedLongInRange(sign_mismatch_timeout_ms, value, 0.0f, 10000.0f)) return false;
  }
  else if (strcmp(name, "side_mismatch_fault_enabled") == 0) side_mismatch_fault_enabled = value != 0.0f;
  else if (strcmp(name, "side_mismatch_warn_tps") == 0) {
    if (!assignFloatInRange(side_mismatch_warn_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "side_mismatch_fault_tps") == 0) {
    if (!assignFloatInRange(side_mismatch_fault_tps, value, 0.0f, 50000.0f)) return false;
  }
  else if (strcmp(name, "encoder_jump_fault_enabled") == 0) encoder_jump_fault_enabled = value != 0.0f;
  else if (strcmp(name, "encoder_jump_tps") == 0) {
    if (!assignFloatInRange(encoder_jump_tps, value, 0.0f, 100000.0f)) return false;
  }
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
    if (commandHasExtraTokens()) {
      sendControlAck("parse", "bad_status");
      return;
    }
    sendStatus();
    sendParamDump();
    return;
  }

  if (strcmp(verb, "STATUS_STREAM") == 0) {
    char* value_text = strtok(NULL, " ");
    if (value_text == NULL) {
      return;
    }
    int requested_stream = 0;
    if (!parseIntToken(value_text, requested_stream) || commandHasExtraTokens()) {
      sendControlAck("status_stream", "bad_value");
      return;
    }
    status_stream_enabled = requested_stream != 0;
    sendControlAck("status_stream", status_stream_enabled ? "on" : "off");
    return;
  }

  if (strcmp(verb, "PARAMDUMP") == 0) {
    if (commandHasExtraTokens()) {
      sendControlAck("parse", "bad_paramdump");
      return;
    }
    sendParamDump();
    return;
  }

  if (strcmp(verb, "V") == 0) {
    char* v_text = strtok(NULL, " ");
    char* omega_text = strtok(NULL, " ");
    if (v_text == NULL || omega_text == NULL) {
      last_command_ms = millis();
      stopController("bad_velocity", true);
      return;
    }
    last_command_ms = millis();
    float requested_v = 0.0f;
    float requested_omega = 0.0f;
    if (
      !parseFiniteFloatToken(v_text, requested_v) ||
      !parseFiniteFloatToken(omega_text, requested_omega) ||
      commandHasExtraTokens()
    ) {
      stopController("bad_velocity", true);
      return;
    }
    if (controller_mode == MODE_FAULT || strcmp(fault_reason, "none") != 0) {
      return;
    }
    target_v_mps = requested_v;
    target_omega_radps = requested_omega;
    clearFault();
    controller_mode = MODE_VELOCITY;
    return;
  }

  if (strcmp(verb, "RAW2") == 0) {
    char* left_text = strtok(NULL, " ");
    char* right_text = strtok(NULL, " ");
    if (left_text == NULL || right_text == NULL) {
      last_command_ms = millis();
      stopController("bad_raw2", true);
      return;
    }
    last_command_ms = millis();
    int requested_left_pwm = DEFAULT_PWM_NEUTRAL_US;
    int requested_right_pwm = DEFAULT_PWM_NEUTRAL_US;
    if (
      !parseIntToken(left_text, requested_left_pwm) ||
      !parseIntToken(right_text, requested_right_pwm) ||
      commandHasExtraTokens()
    ) {
      stopController("bad_raw2", true);
      return;
    }
    if (controller_mode == MODE_FAULT || strcmp(fault_reason, "none") != 0) {
      return;
    }
    target_left_pwm = clampPwm(requested_left_pwm);
    target_right_pwm = clampPwm(requested_right_pwm);
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
    float value = 0.0f;
    bool ok = parseFiniteFloatToken(value_text, value) && !commandHasExtraTokens() && setParam(name, value);
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

void processTransport(
  Stream& stream,
  char* buf,
  const int buf_len,
  int& buf_idx,
  bool& buf_overflow,
  const char* transport_name
) {
  while (stream.available()) {
    char c = stream.read();
    if (c == '\n') {
      if (buf_overflow) {
        sendControlAck("parse", "overflow");
        buf_overflow = false;
      } else if (buf_idx > 0) {
        buf[buf_idx] = '\0';
        parseCommand(buf, transport_name);
      }
      buf_idx = 0;
    } else if (c != '\r') {
      if (buf_overflow) {
        continue;
      }
      if (buf_idx < buf_len - 1) {
        buf[buf_idx++] = c;
      } else {
        buf_idx = 0;
        buf_overflow = true;
      }
    }
  }
}

void updateHeartbeatLed(unsigned long now_ms) {
  if (now_ms - last_heartbeat_led_ms < HEARTBEAT_LED_INTERVAL_MS) {
    return;
  }
  last_heartbeat_led_ms = now_ms;
  heartbeat_led_on = !heartbeat_led_on;
  digitalWrite(HEARTBEAT_LED_PIN, heartbeat_led_on ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  motor_left.attach(PWM_L, 1000, 2000);
  motor_right.attach(PWM_R, 1000, 2000);
  applyNeutralNow();

  last_control_ms = millis();
  last_status_ms = millis();
  last_command_ms = millis();
  last_heartbeat_led_ms = millis();
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
    Serial.print("DBG firmware ");
    Serial.println(FIRMWARE_ID);
    Serial.println("DBG PID backend QuickPID");
  }
}

void loop() {
  if (Serial) {
    processTransport(Serial, usb_buf, COMMAND_BUFFER_LEN, usb_buf_idx, usb_buf_overflow, "usb");
  }
  processTransport(Serial1, uart_buf, COMMAND_BUFFER_LEN, uart_buf_idx, uart_buf_overflow, "uart");

  unsigned long now_ms = millis();
  updateHeartbeatLed(now_ms);

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
