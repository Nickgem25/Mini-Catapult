/*
  SERVO TEST — MG90S only
  WIRING:
    Signal -> GPIO 13
    VCC    -> 5V (external supply recommended)
    GND    -> common ground
  LIBRARY: ESP32Servo

  Behavior:
    ENGAGE (0 deg): spring does the work, so we detach after moving
                    there -- no need to hold power/position.
    DISENGAGE (30 deg): must actively hold against the spring, so we
                    stay attached. We also re-home to 0 first before
                    every disengage move, so each cycle starts from
                    the same known reference point instead of
                    whatever position the servo drifted to while
                    detached -- this is what prevents drift building
                    up over repeated cycles.
*/

#include <Arduino.h>
#include <ESP32Servo.h>

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

//  ======  SETUP   ======
void setup() {
    Serial.begin(115200);
    delay(300);
    //Servo
    engageClutch(); // start at rest    

    //Motor
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH); // enable driver
}

void loop() {
    engageClutch();
    delay(1000);    

    motorForward();
    delay(2000);

    motorStop();
    delay(1000);

    disengageClutch();
    delay(1000);
}
