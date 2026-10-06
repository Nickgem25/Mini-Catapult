/*
  MOTOR TEST — N20 via DRV8833 only
  WIRING:
    AIN1 -> GPIO 26
    AIN2 -> GPIO 27
    SLP  -> GPIO 25
    VM   -> external motor power supply
    GND  -> common ground
    AOUT1/AOUT2 -> N20 motor leads
*/

#include <Arduino.h>

const int AIN1 = 26;
const int AIN2 = 27;
const int SLP  = 25;
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;
const int MOTOR_SPEED = 255; // 0-255
const int AIN1_CHANNEL = 0;
const int AIN2_CHANNEL = 1;

void motorForward() {
    ledcWrite(AIN1_CHANNEL, MOTOR_SPEED);
    ledcWrite(AIN2_CHANNEL, 0);
}

void motorReverse() {
    ledcWrite(AIN1_CHANNEL, 0);
    ledcWrite(AIN2_CHANNEL, MOTOR_SPEED);
}

void motorStop() {
    ledcWrite(AIN1_CHANNEL, 0);
    ledcWrite(AIN2_CHANNEL, 0);
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("Motor test starting...");

    ledcSetup(AIN1_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(AIN2_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(AIN1, AIN1_CHANNEL);
    ledcAttachPin(AIN2, AIN2_CHANNEL);
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH); // enable driver
}

void loop() {
    motorForward();
}
