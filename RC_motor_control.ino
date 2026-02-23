// ============================================================
// RC GO-KART FULL CONTROL
// Arduino Mega 2560 + RP3 (CRSF)
//
// RIGHT STICK LEFT/RIGHT = steering (Talon SRX, brushed 12V)
// PINS 2/3 = quadrature encoder on steering (position feedback)
// RIGHT STICK UP         = forward throttle (BLDC 48V)
// RIGHT STICK DOWN       = reverse throttle
// SD SWITCH (hold)       = brake + kill throttle
// SIGNAL LOST            = brake + kill throttle
//
// Upload BEFORE turning on any power
// ============================================================

#include <Servo.h>

// ----- CRSF -----
#define CRSF_SERIAL           Serial1
#define CRSF_BAUDRATE         420800
#define CRSF_SYNC_BYTE        0xC8
#define CRSF_FRAMETYPE_RC     0x16
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_NUM_CHANNELS     16

// ----- PINS -----
#define THROTTLE_PIN    6     // EZkontrol throttle signal
#define REVERSE_PIN     4     // EZkontrol reverse signal
#define BRAKE_PIN       5     // Brake L white wire (LOW = brake ON)
#define STEERING_PIN    9     // Talon SRX PWM signal
#define ENCODER_A_PIN   2     // Steering quadrature encoder A
#define ENCODER_B_PIN   3     // Steering quadrature encoder B

// ----- CHANNELS (0-indexed) -----
#define THROTTLE_CH     1     // CH2 = right stick up/down
#define STEERING_CH     0     // CH1 = right stick left/right
#define BRAKE_CH        6     // CH7 = SD momentary switch

// ----- CRSF RANGE -----
#define CRSF_CENTER     992
#define CRSF_MIN        172
#define CRSF_MAX        1811

// ----- THROTTLE TUNING -----
#define INVERT_DIRECTION      true
#define DEADBAND              30
#define MAX_PWM               51
#define MIN_PWM               30
#define RAMP_STEP             1
#define FAILSAFE_MS           500

// ----- STEERING TUNING -----
// Talon SRX expects 1000-2000us PWM, 1500us = stop
#define STEERING_CENTER_US    1500
#define STEERING_MAX_US       2000
#define STEERING_MIN_US       1000
#define STEERING_DEADBAND     20    // CRSF units around center

// ----- BRAKE SWITCH THRESHOLD -----
#define BRAKE_SWITCH_THRESHOLD  1000

// ============================================================
uint16_t channels[CRSF_NUM_CHANNELS];
uint8_t  crsfBuf[CRSF_MAX_PACKET_LEN];
uint8_t  crsfBufIndex = 0;
uint8_t  crsfPacketLen = 0;

unsigned long lastValidPacketMs = 0;
bool failsafeActive = true;

int currentPWM = 0;
int targetPWM = 0;
bool reverseActive = false;

unsigned long lastPrintMs = 0;

Servo steeringServo;

// ----- STEERING ENCODER (quadrature on A/B) -----
volatile long encoderCount = 0;

void isrEncoderA() {
    bool A = digitalRead(ENCODER_A_PIN);
    bool B = digitalRead(ENCODER_B_PIN);
    encoderCount += (A == B) ? +1 : -1;
}

void isrEncoderB() {
    bool A = digitalRead(ENCODER_A_PIN);
    bool B = digitalRead(ENCODER_B_PIN);
    encoderCount += (A != B) ? +1 : -1;
}

// ============================================================
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

void decodeChannels(uint8_t *payload) {
    channels[0]  = ((uint16_t)payload[0]       | (uint16_t)payload[1]  << 8)  & 0x07FF;
    channels[1]  = ((uint16_t)payload[1]  >> 3  | (uint16_t)payload[2]  << 5)  & 0x07FF;
    channels[2]  = ((uint16_t)payload[2]  >> 6  | (uint16_t)payload[3]  << 2  | (uint16_t)payload[4]  << 10) & 0x07FF;
    channels[3]  = ((uint16_t)payload[4]  >> 1  | (uint16_t)payload[5]  << 7)  & 0x07FF;
    channels[4]  = ((uint16_t)payload[5]  >> 4  | (uint16_t)payload[6]  << 4)  & 0x07FF;
    channels[5]  = ((uint16_t)payload[6]  >> 7  | (uint16_t)payload[7]  << 1  | (uint16_t)payload[8]  << 9)  & 0x07FF;
    channels[6]  = ((uint16_t)payload[8]  >> 2  | (uint16_t)payload[9]  << 6)  & 0x07FF;
    channels[7]  = ((uint16_t)payload[9]  >> 5  | (uint16_t)payload[10] << 3)  & 0x07FF;
    channels[8]  = ((uint16_t)payload[11]       | (uint16_t)payload[12] << 8)  & 0x07FF;
    channels[9]  = ((uint16_t)payload[12] >> 3  | (uint16_t)payload[13] << 5)  & 0x07FF;
    channels[10] = ((uint16_t)payload[13] >> 6  | (uint16_t)payload[14] << 2  | (uint16_t)payload[15] << 10) & 0x07FF;
    channels[11] = ((uint16_t)payload[15] >> 1  | (uint16_t)payload[16] << 7)  & 0x07FF;
    channels[12] = ((uint16_t)payload[16] >> 4  | (uint16_t)payload[17] << 4)  & 0x07FF;
    channels[13] = ((uint16_t)payload[17] >> 7  | (uint16_t)payload[18] << 1  | (uint16_t)payload[19] << 9)  & 0x07FF;
    channels[14] = ((uint16_t)payload[19] >> 2  | (uint16_t)payload[20] << 6)  & 0x07FF;
    channels[15] = ((uint16_t)payload[20] >> 5  | (uint16_t)payload[21] << 3)  & 0x07FF;
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

// ============================================================
void engageBrake(const char* reason) {
    currentPWM = 0;
    targetPWM = 0;
    analogWrite(THROTTLE_PIN, 0);
    reverseActive = false;
    digitalWrite(REVERSE_PIN, LOW);
    digitalWrite(BRAKE_PIN, LOW);       // Brake L active LOW
    steeringServo.writeMicroseconds(STEERING_CENTER_US);  // Center steering

    if (millis() - lastPrintMs >= 200) {
        lastPrintMs = millis();
        Serial.print("[BRAKE] ");
        Serial.println(reason);
    }
}

void releaseBrake() {
    digitalWrite(BRAKE_PIN, HIGH);
}

// ============================================================
void setup() {
    pinMode(THROTTLE_PIN, OUTPUT);
    digitalWrite(THROTTLE_PIN, LOW);
    analogWrite(THROTTLE_PIN, 0);

    pinMode(REVERSE_PIN, OUTPUT);
    digitalWrite(REVERSE_PIN, LOW);

    pinMode(BRAKE_PIN, OUTPUT);
    digitalWrite(BRAKE_PIN, LOW);       // Brake ON at startup until signal acquired

    steeringServo.attach(STEERING_PIN);
    steeringServo.writeMicroseconds(STEERING_CENTER_US);  // Center on boot

    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), isrEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), isrEncoderB, CHANGE);

    Serial.begin(115200);
    CRSF_SERIAL.begin(CRSF_BAUDRATE);

    Serial.println("=== RC GO-KART FULL CONTROL ===");
    Serial.println("Brake ON until CRSF signal acquired.");
    Serial.println("Right stick UP/DOWN  = throttle/reverse");
    Serial.println("Right stick L/R      = steering (encoder on D2/D3)");
    Serial.println("SD switch (hold)     = brake");
    Serial.println("Signal lost          = brake");
    Serial.print("Max throttle PWM: "); Serial.print(MAX_PWM);
    Serial.print(" (~"); Serial.print((MAX_PWM / 255.0) * 5.0, 2);
    Serial.println("V)");
    Serial.println("Waiting for CRSF...");
}

// ============================================================
void loop() {
    readCrsf();

    // --- FAILSAFE ---
    if (millis() - lastValidPacketMs > FAILSAFE_MS) {
        if (!failsafeActive) {
            failsafeActive = true;
            Serial.println("[FAILSAFE] Signal lost!");
        }
        engageBrake("FAILSAFE - signal lost");
        return;
    }

    // --- SD BRAKE SWITCH ---
    bool brakeSwPressed = (channels[BRAKE_CH] > BRAKE_SWITCH_THRESHOLD);
    if (brakeSwPressed) {
        engageBrake("SD switch held");
        return;
    }

    // --- Normal operation: release brake ---
    releaseBrake();

    // ---- STEERING ----
    int steerRaw = channels[STEERING_CH];
    if (steerRaw >= CRSF_MIN && steerRaw <= CRSF_MAX) {
        int steerOffset = steerRaw - CRSF_CENTER;

        if (abs(steerOffset) <= STEERING_DEADBAND) {
            steeringServo.writeMicroseconds(STEERING_CENTER_US);
        } else {
            // Map CRSF range to Talon SRX PWM range
            int steerUs = map(steerRaw, CRSF_MIN, CRSF_MAX, STEERING_MIN_US, STEERING_MAX_US);
            steerUs = constrain(steerUs, STEERING_MIN_US, STEERING_MAX_US);
            steeringServo.writeMicroseconds(steerUs);
        }
    }

    // ---- THROTTLE ----
    int raw = channels[THROTTLE_CH];
    if (raw < CRSF_MIN || raw > CRSF_MAX) return;

    int offset = raw - CRSF_CENTER;
    if (INVERT_DIRECTION) offset = -offset;

    bool wantForward = (offset > DEADBAND);
    bool wantReverse = (offset < -DEADBAND);

    // Deadband: coast to stop
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
        if (millis() - lastPrintMs >= 200) {
            lastPrintMs = millis();
            long enc;
            noInterrupts();
            enc = encoderCount;
            interrupts();
            Serial.print("CH2="); Serial.print(raw);
            Serial.print("  [CENTER]  CH1="); Serial.print(steerRaw);
            Serial.print("  enc="); Serial.println(enc);
        }
        return;
    }

    // Direction change: ramp to zero first
    if (wantReverse != reverseActive) {
        if (currentPWM > 0) {
            currentPWM = max(currentPWM - RAMP_STEP, 0);
            analogWrite(THROTTLE_PIN, currentPWM);
            return;
        }
        reverseActive = wantReverse;
        digitalWrite(REVERSE_PIN, reverseActive ? HIGH : LOW);
    }

    // Map stick to PWM
    int magnitude = abs(offset) - DEADBAND;
    int maxOffset = (offset > 0) ? (CRSF_MAX - CRSF_CENTER) - DEADBAND
                                 : (CRSF_CENTER - CRSF_MIN) - DEADBAND;

    targetPWM = map(magnitude, 0, maxOffset, MIN_PWM, MAX_PWM);
    targetPWM = constrain(targetPWM, MIN_PWM, MAX_PWM);

    if (currentPWM < targetPWM) currentPWM = min(currentPWM + RAMP_STEP, targetPWM);
    else if (currentPWM > targetPWM) currentPWM = max(currentPWM - RAMP_STEP, targetPWM);

    analogWrite(THROTTLE_PIN, currentPWM);

    // Debug
    if (millis() - lastPrintMs >= 200) {
        lastPrintMs = millis();
        long enc;
        noInterrupts();
        enc = encoderCount;
        interrupts();
        Serial.print("CH2="); Serial.print(raw);
        Serial.print("  dir="); Serial.print(reverseActive ? "REV" : "FWD");
        Serial.print("  tgt="); Serial.print(targetPWM);
        Serial.print("  cur="); Serial.print(currentPWM);
        Serial.print("  ~"); Serial.print((currentPWM / 255.0) * 5.0, 2);
        Serial.print("V  CH1="); Serial.print(steerRaw);
        Serial.print("  enc="); Serial.print(enc);
        Serial.print("  CH7="); Serial.println(channels[BRAKE_CH]);
    }
}