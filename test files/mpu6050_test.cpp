/*
  MPU6050/MPU6500 angle tracking test

  This version does not use a library. It reads raw accel + gyro values and
  combines them into a more stable angle estimate for fast-moving motion.

  WIRING:
    SDA -> GPIO 21
    SCL -> GPIO 22
    VCC -> 3.3V
    GND -> GND
*/

#include <Arduino.h>
#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

// Same register layout on MPU6050 and MPU6500
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
}

void loop()
{
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

    Serial.print("AX:");
    Serial.print(ax, 2);
    Serial.print(" AY:");
    Serial.print(ay, 2);
    Serial.print(" AZ:");
    Serial.print(az, 2);
    Serial.print(" | GYROY:");
    Serial.print(gyroY, 2);
    Serial.print(" deg/s | ANGLE:");
    Serial.print(filteredAngleDeg, 2);
    Serial.print(" deg | ACCEL_ANGLE:");
    Serial.println(accelAngleDeg, 2);

    delay(20);
}