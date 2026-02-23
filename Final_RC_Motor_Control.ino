// ============================================================
// FINAL RC CONTROL: STEERING + THROTTLE/REVERSE
// Arduino Mega 2560 + ELRS RP3 (CRSF)
//
// CH1 (left stick L/R)  -> Talon SRX steering motor (Servo PWM)
// CH2 (right stick U/D) -> EZkontrol throttle + reverse
//
// Upload BEFORE turning on 48V power.
// ============================================================

#include <Servo.h>

// ================= CRSF =================
#define CRSF_SERIAL           Serial1
#define CRSF_BAUDRATE         420800

#define CRSF_SYNC_BYTE        0xC8
#define CRSF_FRAMETYPE_RC     0x16
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_NUM_CHANNELS     16

#define CRSF_CENTER           992
#define CRSF_MIN              172
#define CRSF_MAX              1811

uint16_t channels[CRSF_NUM_CHANNELS];
uint8_t  crsfBuf[CRSF_MAX_PACKET_LEN];
uint8_t  crsfBufIndex  = 0;
uint8_t  crsfPacketLen = 0;

unsigned long lastValidPacketMs = 0;
bool failsafeActive = true;

// ================= STEERING (Talon SRX, Servo PWM) =================
// CH1 = index 0, left stick left/right
Servo talonSRX;
const int TALON_PIN        = 9;
const int STEER_CH         = 0;

const int TALON_NEUTRAL_US = 1500;
const int TALON_MIN_US     = 1000;
const int TALON_MAX_US     = 2000;

const int STEER_DEADZONE         = 10;
const int TALON_MIN_OFFSET_US    = 100;
const int STEER_CMD_CLAMP        = 900;
const bool INVERT_STEERING       = false;

// ================= THROTTLE + REVERSE (EZkontrol) =================
// CH2 = index 1, right stick up/down
const int THROTTLE_PIN     = 6;
const int REVERSE_PIN      = 4;
const int THROTTLE_CH      = 1;

const bool INVERT_DIRECTION = true;
const int THROTTLE_DEADBAND = 30;
const int THROTTLE_MAX_PWM  = 51;
const int THROTTLE_MIN_PWM  = 30;
const int RAMP_STEP         = 1;

int  currentPWM    = 0;
int  targetPWM     = 0;
bool reverseActive = false;

// ================= SHARED =================
const uint32_t FAILSAFE_MS = 500;

// ================= HELPERS =================
static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline int abs_i(int v) { return v < 0 ? -v : v; }

// ================= CRSF CRC8 =================
uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
  static const uint8_t crc8tab[256] = {
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9
  };
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) crc = crc8tab[crc ^ data[i]];
  return crc;
}

// ================= CRSF DECODE =================
void decodeChannels(uint8_t *payload) {
  channels[0]  = ((uint16_t)payload[0]       | (uint16_t)payload[1]  << 8)  & 0x07FF;
  channels[1]  = ((uint16_t)payload[1]  >> 3 | (uint16_t)payload[2]  << 5)  & 0x07FF;
  channels[2]  = ((uint16_t)payload[2]  >> 6 | (uint16_t)payload[3]  << 2  | (uint16_t)payload[4]  << 10) & 0x07FF;
  channels[3]  = ((uint16_t)payload[4]  >> 1 | (uint16_t)payload[5]  << 7)  & 0x07FF;
  channels[4]  = ((uint16_t)payload[5]  >> 4 | (uint16_t)payload[6]  << 4)  & 0x07FF;
  channels[5]  = ((uint16_t)payload[6]  >> 7 | (uint16_t)payload[7]  << 1  | (uint16_t)payload[8]  << 9)  & 0x07FF;
  channels[6]  = ((uint16_t)payload[8]  >> 2 | (uint16_t)payload[9]  << 6)  & 0x07FF;
  channels[7]  = ((uint16_t)payload[9]  >> 5 | (uint16_t)payload[10] << 3)  & 0x07FF;
  channels[8]  = ((uint16_t)payload[11]      | (uint16_t)payload[12] << 8)  & 0x07FF;
  channels[9]  = ((uint16_t)payload[12] >> 3 | (uint16_t)payload[13] << 5)  & 0x07FF;
  channels[10] = ((uint16_t)payload[13] >> 6 | (uint16_t)payload[14] << 2  | (uint16_t)payload[15] << 10) & 0x07FF;
  channels[11] = ((uint16_t)payload[15] >> 1 | (uint16_t)payload[16] << 7)  & 0x07FF;
  channels[12] = ((uint16_t)payload[16] >> 4 | (uint16_t)payload[17] << 4)  & 0x07FF;
  channels[13] = ((uint16_t)payload[17] >> 7 | (uint16_t)payload[18] << 1  | (uint16_t)payload[19] << 9)  & 0x07FF;
  channels[14] = ((uint16_t)payload[19] >> 2 | (uint16_t)payload[20] << 6)  & 0x07FF;
  channels[15] = ((uint16_t)payload[20] >> 5 | (uint16_t)payload[21] << 3)  & 0x07FF;
}

void processCrsfPacket(uint8_t *packet, uint8_t len) {
  if (packet[2] != CRSF_FRAMETYPE_RC) return;
  uint8_t crc = crsf_crc8(&packet[2], crsfPacketLen - 1);
  if (crc != packet[crsfPacketLen + 1]) return;
  decodeChannels(&packet[3]);
  lastValidPacketMs = millis();
  failsafeActive = false;
}

void readCrsf() {
  while (CRSF_SERIAL.available()) {
    uint8_t b = CRSF_SERIAL.read();
    if (crsfBufIndex == 0 && b != CRSF_SYNC_BYTE) continue;
    crsfBuf[crsfBufIndex++] = b;
    if (crsfBufIndex == 2) {
      crsfPacketLen = crsfBuf[1];
      if (crsfPacketLen > CRSF_MAX_PACKET_LEN - 2) {
        crsfBufIndex = 0;
        continue;
      }
    }
    if (crsfBufIndex > 2 && crsfBufIndex == crsfPacketLen + 2) {
      processCrsfPacket(crsfBuf, crsfBufIndex);
      crsfBufIndex = 0;
    }
    if (crsfBufIndex >= CRSF_MAX_PACKET_LEN) crsfBufIndex = 0;
  }
}

// ================= STEERING (Talon SRX) =================
void steerStop() {
  talonSRX.writeMicroseconds(TALON_NEUTRAL_US);
}

void steerDrive(int cmd) {
  int mag = abs_i(cmd);

  if (mag <= STEER_DEADZONE) {
    steerStop();
    return;
  }

  int offsetUs = map(mag, STEER_DEADZONE, STEER_CMD_CLAMP, TALON_MIN_OFFSET_US, 500);
  offsetUs = clampi(offsetUs, 0, 500);

  int pulseUs;
  if (cmd > 0) {
    pulseUs = TALON_NEUTRAL_US + offsetUs;
  } else {
    pulseUs = TALON_NEUTRAL_US - offsetUs;
  }

  pulseUs = clampi(pulseUs, TALON_MIN_US, TALON_MAX_US);
  talonSRX.writeMicroseconds(pulseUs);
}

int steeringToSigned(uint16_t ch) {
  int s = (int)ch - CRSF_CENTER;
  if (INVERT_STEERING) s = -s;
  s = clampi(s, -STEER_CMD_CLAMP, STEER_CMD_CLAMP);
  return s;
}

// ================= THROTTLE + REVERSE (EZkontrol) =================
void throttleStop() {
  currentPWM = 0;
  targetPWM  = 0;
  analogWrite(THROTTLE_PIN, 0);
  digitalWrite(REVERSE_PIN, LOW);
  reverseActive = false;
}

void updateThrottle() {
  int raw = channels[THROTTLE_CH];
  if (raw < CRSF_MIN || raw > CRSF_MAX) return;

  int offset = raw - CRSF_CENTER;
  if (INVERT_DIRECTION) offset = -offset;

  bool wantForward = (offset > THROTTLE_DEADBAND);
  bool wantReverse = (offset < -THROTTLE_DEADBAND);

  // ---- Centered: ramp down to stop ----
  if (!wantForward && !wantReverse) {
    targetPWM = 0;
    if (currentPWM > 0) {
      currentPWM = max(currentPWM - RAMP_STEP, 0);
      analogWrite(THROTTLE_PIN, currentPWM);
    }
    if (currentPWM == 0 && reverseActive) {
      reverseActive = false;
      digitalWrite(REVERSE_PIN, LOW);
    }
    return;
  }

  // ---- Direction change: ramp down first, then switch ----
  if (wantReverse != reverseActive) {
    if (currentPWM > 0) {
      currentPWM = max(currentPWM - RAMP_STEP, 0);
      analogWrite(THROTTLE_PIN, currentPWM);
      return;
    }
    reverseActive = wantReverse;
    digitalWrite(REVERSE_PIN, reverseActive ? HIGH : LOW);
  }

  // ---- Compute target PWM ----
  int magnitude = abs(offset) - THROTTLE_DEADBAND;
  int maxOffset = (offset > 0) ? (CRSF_MAX - CRSF_CENTER) - THROTTLE_DEADBAND
                                : (CRSF_CENTER - CRSF_MIN) - THROTTLE_DEADBAND;

  targetPWM = map(magnitude, 0, maxOffset, THROTTLE_MIN_PWM, THROTTLE_MAX_PWM);
  targetPWM = constrain(targetPWM, THROTTLE_MIN_PWM, THROTTLE_MAX_PWM);

  // ---- Ramp toward target ----
  if (currentPWM < targetPWM) currentPWM = min(currentPWM + RAMP_STEP, targetPWM);
  else if (currentPWM > targetPWM) currentPWM = max(currentPWM - RAMP_STEP, targetPWM);

  analogWrite(THROTTLE_PIN, currentPWM);
}

// ================= FAILSAFE =================
void failsafeStop() {
  steerStop();
  throttleStop();
}

// ================= SETUP =================
void setup() {
  // Throttle/reverse pins safe FIRST
  pinMode(THROTTLE_PIN, OUTPUT);
  digitalWrite(THROTTLE_PIN, LOW);
  analogWrite(THROTTLE_PIN, 0);
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN, LOW);

  // Steering servo
  talonSRX.attach(TALON_PIN);
  steerStop();

  // Serial
  Serial.begin(115200);
  CRSF_SERIAL.begin(CRSF_BAUDRATE);

  Serial.println("============================================");
  Serial.println("  FINAL RC: STEERING + THROTTLE/REVERSE");
  Serial.println("============================================");
  Serial.println("CH1 (L stick L/R) -> Talon SRX steering");
  Serial.println("CH2 (R stick U/D) -> EZkontrol throttle");
  Serial.println("All outputs ZERO. Safe to power on 48V.");
  Serial.print("Throttle max PWM: "); Serial.print(THROTTLE_MAX_PWM);
  Serial.print(" (~"); Serial.print((THROTTLE_MAX_PWM / 255.0) * 5.0, 2);
  Serial.println("V)");
  Serial.println("Waiting for CRSF...");
}

// ================= LOOP =================
unsigned long lastPrintMs = 0;

void loop() {
  readCrsf();

  // ---- Failsafe ----
  if (millis() - lastValidPacketMs > FAILSAFE_MS) {
    if (!failsafeActive) {
      failsafeActive = true;
      failsafeStop();
      Serial.println("[FAILSAFE] Signal lost! All motors stopped.");
    }
    return;
  }

  // ---- Steering (runs every loop, no ramp needed) ----
  int steerCmd = steeringToSigned(channels[STEER_CH]);
  steerDrive(steerCmd);

  // ---- Throttle (ramp logic inside) ----
  updateThrottle();

  // ---- Debug (every 200ms) ----
  if (millis() - lastPrintMs >= 200) {
    lastPrintMs = millis();

    // Steering info
    int steerPulse = TALON_NEUTRAL_US;
    if (abs_i(steerCmd) > STEER_DEADZONE) {
      int off = (int)map(abs_i(steerCmd), STEER_DEADZONE, STEER_CMD_CLAMP, TALON_MIN_OFFSET_US, 500);
      steerPulse = (steerCmd > 0) ? TALON_NEUTRAL_US + off : TALON_NEUTRAL_US - off;
    }
    Serial.print("STEER: CH1="); Serial.print(channels[STEER_CH]);
    Serial.print(" cmd="); Serial.print(steerCmd);
    Serial.print(" us="); Serial.print(steerPulse);

    // Throttle info
    Serial.print("  |  THROT: CH2="); Serial.print(channels[THROTTLE_CH]);
    Serial.print(" dir="); Serial.print(reverseActive ? "REV" : "FWD");
    Serial.print(" tgt="); Serial.print(targetPWM);
    Serial.print(" cur="); Serial.print(currentPWM);
    Serial.print(" ~"); Serial.print((currentPWM / 255.0) * 5.0, 2);
    Serial.println("V");
  }
}
