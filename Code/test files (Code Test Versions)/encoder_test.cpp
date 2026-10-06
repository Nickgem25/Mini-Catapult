/*
  ROTARY ENCODER TEST — encoder + button only
  Uses the ESP32's hardware pulse counter (PCNT) via ESP32Encoder
  library, instead of hand-rolled interrupts. Much more resistant
  to electrical noise than software debouncing alone.

  WIRING:
    CLK -> GPIO 32
    DT  -> GPIO 33
    SW  -> GPIO 4
    VCC -> 3.3V
    GND -> GND

  LIBRARY: ESP32Encoder (by madhephaestus) - add to platformio.ini:
    madhephaestus/ESP32Encoder@^0.11.7
*/

#include <Arduino.h>
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

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("Encoder test starting (hardware PCNT mode)...");

    pinMode(ENC_SW, INPUT_PULLUP);

    // Use the ESP32's internal pull-ups so we don't need external resistors
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // "Half quad" = 1 count per detent click on most common encoder modules.
    // If counts still feel off (e.g. 2 per click), try attachFullQuad instead.
    encoder.attachHalfQuad(ENC_DT, ENC_CLK);
    encoder.setCount(0);
}

int clampAngle(long rawCount) {
    if (rawCount < ANGLE_MIN) return ANGLE_MIN;
    if (rawCount > ANGLE_MAX) return ANGLE_MAX;
    return (int)rawCount;
}

void loop() {
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
}