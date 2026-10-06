#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

const int ENC_CLK = 32;
const int ENC_DT  = 33;
const int ENC_SW  = 4;

const int ANGLE_MIN = 0;
const int ANGLE_MAX = 90;
const int RAW_MIN = ANGLE_MIN * 2;
const int RAW_MAX = ANGLE_MAX * 2;

ESP32Encoder encoder;

unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_MS = 500;

int clampAngle(long rawCount) {
    if (rawCount < ANGLE_MIN) return ANGLE_MIN;
    if (rawCount > ANGLE_MAX) return ANGLE_MAX;
    return (int)rawCount;
}

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

const int AIN1 = 26;
const int AIN2 = 27;
const int SLP  = 25;

void motorForward() {
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

const uint8_t MPU_ADDR = 0x68;

// Same register layout on MPU6050 and MPU6500
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_ACCEL_START = 0x3B; // ACCEL_XOUT_H, then accel(6) + temp(2) + gyro(6)
const uint8_t REG_GYRO_START = 0x43;

const float ACCEL_SCALE = 16384.0f; // default +/-2g mode
const float GYRO_SCALE = 131.0f;    // default +/-250 deg/s mode
const float FILTER_ALPHA = 0.90f;   // complementary filter balance

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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
    Serial.println("OLED test starting...");

    Wire.begin(21, 22); // SDA, SCL

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED not found! Check wiring/address.");
        while (1) delay(1000); // halt here so the error is obvious
    }

    Serial.println("OLED found.");

    //Servo
    engageClutch(); // start at rest    

    //Motor
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH); // enable driver

    //Encoder
    pinMode(ENC_SW, INPUT_PULLUP);

    // Use the ESP32's internal pull-ups so we don't need external resistors
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // "Half quad" = 1 count per detent click on most common encoder modules.
    // If counts still feel off (e.g. 2 per click), try attachFullQuad instead.
    encoder.attachHalfQuad(ENC_DT, ENC_CLK);
    encoder.setCount(0);
}

bool isWinding = false;

void loop()
{
    static int lastPrinted = -1;

    long raw = encoder.getCount();
    if (raw >= RAW_MAX) encoder.setCount(RAW_MAX);
    if (raw <= RAW_MIN) encoder.setCount(RAW_MIN);

    int angle = clampAngle(encoder.getCount() / 2);

    if (angle != lastPrinted) {
        Serial.print("Angle: ");
        Serial.println(angle);
        lastPrinted = angle;
    }
    if (digitalRead(ENC_SW) == LOW) {
        unsigned long now = millis();
        if (now - lastButtonPress > DEBOUNCE_MS) {
            lastButtonPress = now;
            Serial.print("Button pressed! Confirmed angle: ");
            Serial.println(angle);
        }
    }

    // 1. Start winding once (e.g., automatically at boot or via trigger)
    if (!isWinding && filteredAngleDeg >= 10.0f) {
        engageClutch();
        motorForward();
        isWinding = true;
    }

    float rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;
    readMPUDataAveraged(rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ, 4);

    float ax = rawAX / ACCEL_SCALE;
    float ay = rawAY / ACCEL_SCALE;
    float az = rawAZ / ACCEL_SCALE;

    // Gyro in deg/sec after removing zero bias.
    float gyroX = (rawGX - gyroXOffset) / GYRO_SCALE;
    float gyroY = (rawGY - gyroYOffset) / GYRO_SCALE;
    float gyroZ = (rawGZ - gyroZOffset) / GYRO_SCALE;

    // For a catapult arm rotating mostly in the X-Z plane, use the X-axis tilt.
    // If your arm rotates the other way, switch to atan2(ay, az) and gyroX.
    float accelAngleDeg = atan2(ax, az) * 180.0f / PI;

    unsigned long nowUs = micros();
    float dt = (nowUs - lastTimeUs) / 1000000.0f;
    lastTimeUs = nowUs;

    if (dt <= 0.0f)
    {
        dt = 0.016f;
    }

    // Integrate gyro rotation to estimate instantaneous angle.
    float gyroAngleDeg = filteredAngleDeg + (gyroY * dt);

    // Sanity check: total accel magnitude should be close to 1g if the
    // sensor is only feeling gravity. During motion, extra force throws
    // this off -- in that case, trust the gyro alone for this sample
    // instead of blending in a corrupted accel angle.
    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    bool accelIsTrustworthy = (accelMagnitude > 0.9f && accelMagnitude < 1.1f);

    if (accelIsTrustworthy)
    {
        // Complementary filter: gyro tracks fast motion, accel corrects drift.
        filteredAngleDeg = (FILTER_ALPHA * gyroAngleDeg) + ((1.0f - FILTER_ALPHA) * accelAngleDeg);
    }
    else
    {
        filteredAngleDeg = gyroAngleDeg;
    }

    // 3. Stop logic — triggers ONLY while winding and angle reaches target
    if (isWinding && filteredAngleDeg < 30.0f) {
        motorStop();
        isWinding = false; // Prevents motor from restarting!

        delay(1000);
        disengageClutch();
        delay(1000);
        engageClutch();
        delay(1000);
    }
    

    // 4. Display Update
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("ANGLE:");
    display.print(filteredAngleDeg, 2);
    display.print("\nMOTOR:");
    display.println(isWinding ? "RUNNING" : "STOPPED");
    display.display();

    delay(20);
}