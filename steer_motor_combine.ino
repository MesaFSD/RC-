#include <Servo.h>

/*
  Encoder:
    A -> pin 2
    B -> pin 3

  Motor controller (ESC/servo-style input):
    signal -> pin 9

  Bang-bang control:
    - If error > deadband: drive forward at +powerPercent
    - If error < -deadband: drive reverse at -powerPercent
    - Else: neutral (stop)

  Serial commands:
    s <counts>   set setpoint in encoder counts (ex: "s 1000")
    z            zero encoder (set current position to 0)
    b <counts>   set deadband in counts (ex: "b 10")
    p <percent>  set power percent 0..100 (ex: "p 35")

  Output (CSV, ~20 Hz):
    <position>,<setpoint>
*/

volatile long encoderCount = 0;

void isrA() {
  bool A = digitalRead(2);
  bool B = digitalRead(3);
  encoderCount += (A == B) ? +1 : -1;
}

void isrB() {
  bool A = digitalRead(2);
  bool B = digitalRead(3);
  encoderCount += (A != B) ? +1 : -1;
}

// ----- Motor controller -----
Servo motor;
const int MOTOR_PIN = 9;
const int PWM_NEUTRAL = 1500;
const int PWM_MIN = 1400;
const int PWM_MAX = 1600;

// ----- Bang-bang params -----
long setpointCounts = 0;
long deadbandCounts = 30;   // stop when |error| <= deadbandCounts
int powerPercent = 35;      // 0..100, how hard to drive when outside deadband

// Timing
const unsigned long CONTROL_PERIOD_US = 5000; // 200 Hz-ish
unsigned long lastControlUs = 0;

long readEncoderAtomic() {
  noInterrupts();
  long c = encoderCount;
  interrupts();
  return c;
}

void writePercentToMotor(int percent) {
  if (percent > 100) percent = 100;
  if (percent < -100) percent = -100;

  int pwm = PWM_NEUTRAL + percent * 5; // -100..100 -> 1000..2000
  if (pwm < PWM_MIN) pwm = PWM_MIN;
  if (pwm > PWM_MAX) pwm = PWM_MAX;

  motor.writeMicroseconds(pwm);
}

void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), isrB, CHANGE);

  motor.attach(MOTOR_PIN);

  // Arm / safety: hold neutral briefly
  motor.writeMicroseconds(PWM_NEUTRAL);
  delay(2000);

  lastControlUs = micros();
}

void handleSerial() {
  if (!Serial.available()) return;

  char cmd = (char)Serial.read();
  if (cmd == '\r' || cmd == '\n' || cmd == ' ') return;

  if (cmd == 's') {
    setpointCounts = Serial.parseInt();
  } else if (cmd == 'z') {
    noInterrupts();
    encoderCount = 0;
    interrupts();
    setpointCounts = 0;
  } else if (cmd == 'b') {
    long v = Serial.parseInt();
    if (v >= 0) deadbandCounts = v;
  } else if (cmd == 'p') {
    long v = Serial.parseInt();
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    powerPercent = (int)v;
  }

  // consume rest of line
  while (Serial.available()) {
    char c = (char)Serial.peek();
    Serial.read();
    if (c == '\n') break;
  }
}

void loop() {
  handleSerial();

  unsigned long nowUs = micros();
  if ((nowUs - lastControlUs) < CONTROL_PERIOD_US) return;
  lastControlUs = nowUs;

  long pos = readEncoderAtomic();
  long error = setpointCounts - pos;

  int cmdPercent = 0;
  if (error > deadbandCounts) {
    cmdPercent = +powerPercent;
  } else if (error < -deadbandCounts) {
    cmdPercent = -powerPercent;
  } else {
    cmdPercent = 0;
  }

  writePercentToMotor(cmdPercent);

  // Print encoder position + setpoint only (~20 Hz)
  static unsigned long lastPrintMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastPrintMs >= 50) {
    lastPrintMs = nowMs;
    Serial.print(pos);
    Serial.print(',');
    Serial.println(setpointCounts);
  }
}