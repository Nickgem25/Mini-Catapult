#include <Arduino.h>
// constants won't change. They're used here to set pin

#include <Arduino.h>
// constants won't change. They're used here to set pin numbers:
#define BUTTON_PIN 13 // ESP32 pin GPIO13, which connected to button
#define LED_PIN 2     // ESP32 pin GPIO2, which connected to led
// variables will be changed:
int led_state = LOW;   // the current state of LED
int button_state;      // the current state of button
int last_button_state; // the previous state of button
void setup()
{
    Serial.begin(9600); // initialize serial
    Serial.println("setup started");
    pinMode(BUTTON_PIN, INPUT); // set ESP32 pin to input mode
    pinMode(LED_PIN, OUTPUT);   // set ESP32 pin to output mode
    button_state = digitalRead(BUTTON_PIN);
}
void loop()
{
    last_button_state = button_state;       // save the last state
    button_state = digitalRead(BUTTON_PIN); // read new state
    if (last_button_state == HIGH && button_state == LOW)
    {
        Serial.println("The button is pressed");
        // toggle state of LED
        led_state = !led_state;
        // control LED arccoding to the toggled state
        digitalWrite(LED_PIN, led_state);
    }
}