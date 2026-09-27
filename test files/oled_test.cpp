/*
  OLED TEST — SSD1306 only
  WIRING:
    SDA -> GPIO 21
    SCL -> GPIO 22
    VCC -> 3.3V
    GND -> GND
  LIBRARIES: Adafruit SSD1306, Adafruit GFX Library
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("OLED test starting...");

    Wire.begin(21, 22); // SDA, SCL

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED not found! Check wiring/address.");
        while (1) delay(1000); // halt here so the error is obvious
    }

    Serial.println("OLED found.");
}

void loop() {
    static int counter = 0;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("OLED TEST OK");
    display.print("Counter: ");
    display.println(counter);
    display.display();

    Serial.print("Displayed counter: ");
    Serial.println(counter);

    counter++;
    delay(1000);
}
