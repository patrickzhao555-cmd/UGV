// Teensy 4.1 - dual-channel motor PWM + 4-wheel encoder bridge
// Serial  = USB serial transport and debug console
// Serial1 = optional Jetson UART on pins 0/1
//
// Jetson -> Teensy:
//   M<left_us>,<right_us>\n
//   Example: M1500,1500
//
// Teensy -> Jetson:
//   E<fl>,<fr>,<rl>,<rr>,<millis>\n
//   Example: E120,118,121,119,5320
//
// The same command/encoder protocol is accepted on USB Serial and Serial1.
// That lets the robot run over /dev/ttyACM0 today and still keep UART support
// available later if you move the wiring back to Serial1.
//
// Left PWM drives both left-side motors.
// Right PWM drives both right-side motors.

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

volatile long enc_fl = 0;
volatile long enc_fr = 0;
volatile long enc_rl = 0;
volatile long enc_rr = 0;

Servo motor_left;
Servo motor_right;

unsigned long last_send_ms = 0;
unsigned long last_command_ms = 0;

const unsigned long SEND_INTERVAL_MS = 20;
const unsigned long COMMAND_TIMEOUT_MS = 250;
const int PWM_MIN_US = 1100;
const int PWM_NEUTRAL_US = 1500;
const int PWM_MAX_US = 1900;

char usb_buf[48];
char uart_buf[48];
int usb_buf_idx = 0;
int uart_buf_idx = 0;
int current_left_pwm = PWM_NEUTRAL_US;
int current_right_pwm = PWM_NEUTRAL_US;

void isr_fl_a() {
  if (digitalRead(ENC_FRONT_LEFT_A) == digitalRead(ENC_FRONT_LEFT_B)) enc_fl--;
  else enc_fl++;
}

void isr_fl_b() {
  if (digitalRead(ENC_FRONT_LEFT_A) == digitalRead(ENC_FRONT_LEFT_B)) enc_fl++;
  else enc_fl--;
}

void isr_fr_a() {
  if (digitalRead(ENC_FRONT_RIGHT_A) == digitalRead(ENC_FRONT_RIGHT_B)) enc_fr++;
  else enc_fr--;
}

void isr_fr_b() {
  if (digitalRead(ENC_FRONT_RIGHT_A) == digitalRead(ENC_FRONT_RIGHT_B)) enc_fr--;
  else enc_fr++;
}

void isr_rl_a() {
  if (digitalRead(ENC_REAR_LEFT_A) == digitalRead(ENC_REAR_LEFT_B)) enc_rl--;
  else enc_rl++;
}

void isr_rl_b() {
  if (digitalRead(ENC_REAR_LEFT_A) == digitalRead(ENC_REAR_LEFT_B)) enc_rl++;
  else enc_rl--;
}

void isr_rr_a() {
  if (digitalRead(ENC_REAR_RIGHT_A) == digitalRead(ENC_REAR_RIGHT_B)) enc_rr++;
  else enc_rr--;
}

void isr_rr_b() {
  if (digitalRead(ENC_REAR_RIGHT_A) == digitalRead(ENC_REAR_RIGHT_B)) enc_rr--;
  else enc_rr++;
}

void writeMotors(int left_us, int right_us) {
  current_left_pwm = constrain(left_us, PWM_MIN_US, PWM_MAX_US);
  current_right_pwm = constrain(right_us, PWM_MIN_US, PWM_MAX_US);
  motor_left.writeMicroseconds(current_left_pwm);
  motor_right.writeMicroseconds(current_right_pwm);
}

void sendProtocolEncoderFrame(Stream& stream, long fl, long fr, long rl, long rr) {
  stream.print("E");
  stream.print(fl); stream.print(",");
  stream.print(fr); stream.print(",");
  stream.print(rl); stream.print(",");
  stream.print(rr); stream.print(",");
  stream.println(millis());
}

void sendEncoderFrame() {
  long fl, fr, rl, rr;

  noInterrupts();
  fl = enc_fl;
  fr = enc_fr;
  rl = enc_rl;
  rr = enc_rr;
  interrupts();

  if (Serial) {
    sendProtocolEncoderFrame(Serial, fl, fr, rl, rr);
  }
  sendProtocolEncoderFrame(Serial1, fl, fr, rl, rr);

  if (Serial) {
    Serial.print("DBG ENC ");
    Serial.print(fl); Serial.print(",");
    Serial.print(fr); Serial.print(",");
    Serial.print(rl); Serial.print(",");
    Serial.print(rr); Serial.print(" pwm=");
    Serial.print(current_left_pwm); Serial.print(",");
    Serial.println(current_right_pwm);
  }
}

void parseCommand(char* s, const char* transport_name) {
  if (s[0] != 'M') return;

  char* comma = strchr(s + 1, ',');
  if (!comma) return;

  *comma = '\0';
  int left_us = atoi(s + 1);
  int right_us = atoi(comma + 1);

  writeMotors(left_us, right_us);
  last_command_ms = millis();

  if (Serial) {
    Serial.print("DBG CMD ");
    Serial.print(transport_name);
    Serial.print(" ");
    Serial.print(current_left_pwm);
    Serial.print(",");
    Serial.println(current_right_pwm);
  }
}

void processTransport(Stream& stream, char* buf, int& buf_idx, const char* transport_name) {
  while (stream.available()) {
    char c = stream.read();
    if (c == '\n') {
      buf[buf_idx] = '\0';
      parseCommand(buf, transport_name);
      buf_idx = 0;
    } else if (c != '\r' && buf_idx < 47) {
      buf[buf_idx++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(ENC_FRONT_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_FRONT_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_FRONT_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_FRONT_RIGHT_B, INPUT_PULLUP);
  pinMode(ENC_REAR_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_REAR_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_REAR_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_REAR_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FRONT_LEFT_A), isr_fl_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FRONT_LEFT_B), isr_fl_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FRONT_RIGHT_A), isr_fr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FRONT_RIGHT_B), isr_fr_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_LEFT_A), isr_rl_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_LEFT_B), isr_rl_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_RIGHT_A), isr_rr_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_REAR_RIGHT_B), isr_rr_b, CHANGE);

  motor_left.attach(PWM_L, 1000, 2000);
  motor_right.attach(PWM_R, 1000, 2000);
  writeMotors(PWM_NEUTRAL_US, PWM_NEUTRAL_US);
  last_command_ms = millis();

  Serial.println("Teensy 4.1 motor bridge ready");
  Serial.println("USB Serial and Serial1 both accept M... commands");
  Serial.println("USB Serial and Serial1 both publish E... encoder frames");
}

void loop() {
  if (Serial) {
    processTransport(Serial, usb_buf, usb_buf_idx, "usb");
  }
  processTransport(Serial1, uart_buf, uart_buf_idx, "uart");

  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    if (current_left_pwm != PWM_NEUTRAL_US || current_right_pwm != PWM_NEUTRAL_US) {
      writeMotors(PWM_NEUTRAL_US, PWM_NEUTRAL_US);
      if (Serial) {
        Serial.println("DBG Failsafe stop");
      }
    }
  }

  if (millis() - last_send_ms >= SEND_INTERVAL_MS) {
    last_send_ms = millis();
    sendEncoderFrame();
  }
}
