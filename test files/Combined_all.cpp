#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

// ---------------- Encoder (angle select dial) ----------------
const int ENC_CLK = 32;
const int ENC_DT  = 33;
const int ENC_SW  = 4;   // confirm target angle -> start winding

const int ANGLE_MIN = 0;
const int ANGLE_MAX = 90;
const int RAW_MIN = ANGLE_MIN * 2;
const int RAW_MAX = ANGLE_MAX * 2;

ESP32Encoder encoder;

unsigned long lastConfirmPress = 0;
const unsigned long DEBOUNCE_MS = 500;

int clampAngle(long rawCount) {
    if (rawCount < ANGLE_MIN) return ANGLE_MIN;
    if (rawCount > ANGLE_MAX) return ANGLE_MAX;
    return (int)rawCount;
}

// ---------------- Launch button ----------------
const int LAUNCH_BTN = 16; // check this pin is free on your specific ESP32 board
unsigned long lastLaunchPress = 0;

// ---------------- MG90S dog clutch ----------------
const int SERVO_PIN = 13;
Servo testServo;

const int ANGLE_ENGAGED    = 0;
const int ANGLE_DISENGAGED = 30;
const int SERVO_MOVE_TIME_MS = 400; // time to physically move + settle

void engageClutch() {
    testServo.attach(SERVO_PIN);
    testServo.write(ANGLE_ENGAGED);
    delay(SERVO_MOVE_TIME_MS);
    testServo.detach(); // spring holds this position, no power needed
}

void disengageClutch() {
    testServo.attach(SERVO_PIN);

    // Home to the known engaged position first, to eliminate any
    // drift that happened while the servo was detached.
    testServo.write(ANGLE_ENGAGED);
    delay(SERVO_MOVE_TIME_MS);

    testServo.write(ANGLE_DISENGAGED);
    delay(SERVO_MOVE_TIME_MS);
    // stay attached -- must keep holding torque against the spring
}

// ---------------- DRV8833 winch motor ----------------
const int AIN1 = 26;
const int AIN2 = 27;
const int SLP  = 25;

void motorForward() { // winds arm down -- flip AIN1/AIN2 if it spins the wrong way
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
}

void motorReverse() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
}

void motorStop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
}

// ---------------- MPU6050/6500 arm angle ----------------
const uint8_t MPU_ADDR = 0x68;

const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_START = 0x3B; // ACCEL_XOUT_H, then accel(6) + temp(2) + gyro(6)
const uint8_t REG_GYRO_START = 0x43;

const float ACCEL_SCALE = 16384.0f; // default +/-2g mode
const float GYRO_SCALE = 131.0f;    // default +/-250 deg/s mode
const float FILTER_ALPHA = 0.90f;   // complementary filter balance

float gyroXOffset = 0.0f;
float gyroYOffset = 0.0f;
float gyroZOffset = 0.0f;
float filteredAngleDeg = 0.0f;
unsigned long lastTimeUs = 0;

void mpuWriteRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

void readMPUData(int16_t &rawAX, int16_t &rawAY, int16_t &rawAZ, int16_t &rawGX, int16_t &rawGY, int16_t &rawGZ)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_START);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    rawAX = (Wire.read() << 8) | Wire.read();
    rawAY = (Wire.read() << 8) | Wire.read();
    rawAZ = (Wire.read() << 8) | Wire.read();
    Wire.read();
    Wire.read(); // temperature bytes
    rawGX = (Wire.read() << 8) | Wire.read();
    rawGY = (Wire.read() << 8) | Wire.read();
    rawGZ = (Wire.read() << 8) | Wire.read();
}

void readMPUDataAveraged(float &avgAX, float &avgAY, float &avgAZ,
                          float &avgGX, float &avgGY, float &avgGZ,
                          int samples)
{
    long sumAX = 0, sumAY = 0, sumAZ = 0;
    long sumGX = 0, sumGY = 0, sumGZ = 0;

    for (int i = 0; i < samples; i++)
    {
        int16_t rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;
        readMPUData(rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ);
        sumAX += rawAX; sumAY += rawAY; sumAZ += rawAZ;
        sumGX += rawGX; sumGY += rawGY; sumGZ += rawGZ;
    }

    avgAX = sumAX / (float)samples;
    avgAY = sumAY / (float)samples;
    avgAZ = sumAZ / (float)samples;
    avgGX = sumGX / (float)samples;
    avgGY = sumGY / (float)samples;
    avgGZ = sumGZ / (float)samples;
}

void calibrateGyroOffsets()
{
    int32_t sumGX = 0;
    int32_t sumGY = 0;
    int32_t sumGZ = 0;

    for (int i = 0; i < 500; i++)
    {
        int16_t rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;
        readMPUData(rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ);
        sumGX += rawGX;
        sumGY += rawGY;
        sumGZ += rawGZ;
        delay(5);
    }

    gyroXOffset = sumGX / 500.0f;
    gyroYOffset = sumGY / 500.0f;
    gyroZOffset = sumGZ / 500.0f;

    Serial.print("Gyro offsets: ");
    Serial.print(gyroXOffset);
    Serial.print(", ");
    Serial.print(gyroYOffset);
    Serial.print(", ");
    Serial.println(gyroZOffset);
}

void updateFilteredAngle()
{
    float rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;
    readMPUDataAveraged(rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ, 4);

    float ax = rawAX / ACCEL_SCALE;
    float ay = rawAY / ACCEL_SCALE;
    float az = rawAZ / ACCEL_SCALE;

    float gyroY = (rawGY - gyroYOffset) / GYRO_SCALE;

    // For a catapult arm rotating mostly in the X-Z plane, use the X-axis tilt.
    // If your arm rotates the other way, switch to atan2(ay, az) and gyroX.
    float accelAngleDeg = atan2(ax, az) * 180.0f / PI;

    unsigned long nowUs = micros();
    float dt = (nowUs - lastTimeUs) / 1000000.0f;
    lastTimeUs = nowUs;
    if (dt <= 0.0f) dt = 0.016f;

    float gyroAngleDeg = filteredAngleDeg + (gyroY * dt);

    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    bool accelIsTrustworthy = (accelMagnitude > 0.9f && accelMagnitude < 1.1f);

    if (accelIsTrustworthy) {
        filteredAngleDeg = (FILTER_ALPHA * gyroAngleDeg) + ((1.0f - FILTER_ALPHA) * accelAngleDeg);
    } else {
        filteredAngleDeg = gyroAngleDeg;
    }
}

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- State machine ----------------
enum CatapultState { SELECT, WINDING, SETTLING, READY };
CatapultState state = SELECT;

int targetAngle = 0; // 0-90, set in SELECT via encoder

// --- SETTLING tuning (adjust NUDGE_DURATION_MS on the bench) ---
const float ANGLE_TOLERANCE_DEG   = 0.5f;
const unsigned long SETTLE_DELAY_MS  = 1500; // wait after stop/nudge before re-checking
const unsigned long NUDGE_DURATION_MS = 45;   // short correction pulse -- TUNE THIS
const int MAX_SETTLE_RETRIES = 10;            // safety escape hatch

bool settleWaiting = false;     // true while waiting out SETTLE_DELAY_MS
unsigned long settleWaitStart = 0;
int settleRetries = 0;

void enterSettling()
{
    motorStop();
    settleWaiting = true;
    settleWaitStart = millis();
    settleRetries = 0;
    state = SETTLING;
}

void enterSelect()
{
    encoder.setCount(0);
    targetAngle = 0;
    state = SELECT;
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Wire.begin(21, 22);

    // Wake the chip up; it defaults to sleep on reset
    mpuWriteRegister(REG_PWR_MGMT_1, 0x00);

    // Let the sensor settle a bit before calibration
    delay(100);
    calibrateGyroOffsets();

    lastTimeUs = micros();
    Serial.println("MPU6500 ready. Using gyro + accel complementary filter.");

    delay(300);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED not found! Check wiring/address.");
        while (1) delay(1000); // halt here so the error is obvious
    }
    Serial.println("OLED found.");

    // Servo -- start engaged/at rest
    engageClutch();

    // Motor
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH); // enable driver

    // Encoder
    pinMode(ENC_SW, INPUT_PULLUP);
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(ENC_DT, ENC_CLK);
    encoder.setCount(0);

    // Launch button
    pinMode(LAUNCH_BTN, INPUT_PULLUP);

    state = SELECT;
}

void loop()
{
    // --- Always keep the arm angle estimate current ---
    updateFilteredAngle();

    // --- Debounced button reads ---
    bool confirmPressed = false;
    if (digitalRead(ENC_SW) == LOW) {
        unsigned long now = millis();
        if (now - lastConfirmPress > DEBOUNCE_MS) {
            lastConfirmPress = now;
            confirmPressed = true;
        }
    }

    bool launchPressed = false;
    if (digitalRead(LAUNCH_BTN) == LOW) {
        unsigned long now = millis();
        if (now - lastLaunchPress > DEBOUNCE_MS) {
            lastLaunchPress = now;
            launchPressed = true;
        }
    }

    // --- State machine ---
    switch (state) {

        case SELECT: {
            // Live-clamp the encoder so it can't run past 0-90
            long raw = encoder.getCount();
            if (raw >= RAW_MAX) encoder.setCount(RAW_MAX);
            if (raw <= RAW_MIN) encoder.setCount(RAW_MIN);

            targetAngle = clampAngle(encoder.getCount() / 2);

            if (confirmPressed) {
                Serial.print("Target confirmed: ");
                Serial.println(targetAngle);
                engageClutch();
                motorForward();
                state = WINDING;
            }
            break;
        }

        case WINDING: {
            // Arm rests high (~85-88 deg) and winch pulls it DOWN toward 0.
            // Once we've wound down to (or past) the chosen target, stop and
            // hand off to SETTLING to fine-tune the exact resting angle.
            if (filteredAngleDeg <= (float)targetAngle) {
                enterSettling();
            }
            break;
        }

        case SETTLING: {
            if (settleWaiting) {
                // Motor is off and we're letting vibration die down so the
                // sensor reading below is trustworthy.
                if (millis() - settleWaitStart >= SETTLE_DELAY_MS) {
                    settleWaiting = false;
                }
                break;
            }

            // Vibration should be settled now -- check how close we really are.
            float error = filteredAngleDeg - (float)targetAngle;

            if (fabs(error) <= ANGLE_TOLERANCE_DEG) {
                Serial.println("Settled within tolerance.");
                state = READY;
            } else if (settleRetries >= MAX_SETTLE_RETRIES) {
                Serial.println("Settle retries exhausted, proceeding anyway.");
                state = READY;
            } else {
                settleRetries++;
                if (error > 0) {
                    // Still above target (not wound down enough) -- nudge more.
                    motorForward();
                } else {
                    // Overshot past target -- nudge back the other way.
                    motorReverse();
                }
                delay(NUDGE_DURATION_MS); // short pulse, deliberately blocking
                motorStop();

                // Wait out another settle period before re-checking.
                settleWaiting = true;
                settleWaitStart = millis();
            }
            break;
        }

        case READY: {
            if (launchPressed) {
                disengageClutch(); // releases the arm
                delay(1000);
                engageClutch();    // reset clutch for next cycle
                enterSelect();
            }
            break;
        }
    }

    // --- OLED ---
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    switch (state) {
        case SELECT:
            display.println("SELECT ANGLE");
            display.print("Target: ");
            display.println(targetAngle);
            display.print("Arm:    ");
            display.println(filteredAngleDeg, 1);
            display.println("Press dial to confirm");
            break;

        case WINDING:
            display.println("WINDING...");
            display.print("Arm:    ");
            display.println(filteredAngleDeg, 1);
            display.print("Target: ");
            display.println(targetAngle);
            break;

        case SETTLING:
            display.println("ALIGNING...");
            display.print("Arm:    ");
            display.println(filteredAngleDeg, 2);
            display.print("Target: ");
            display.println(targetAngle);
            display.print("Retry:  ");
            display.println(settleRetries);
            break;

        case READY:
            display.println("READY TO LAUNCH");
            display.print("Angle: ");
            display.println(filteredAngleDeg, 1);
            display.println("Press launch button");
            break;
    }

    display.display();
    delay(20);
}