#include <Arduino.h>
// constants won't change. They're used here to set pin numbers:
#define BUTTON_PIN 13 // ESP32 pin GPIO13, which connected to button
#define LED_PIN 2     // ESP32 pin GPIO2, which connected to led

int ledstate = LOW; // variable to store the led state
int bigloop = 0;    // variable to store the loop state
int lastButtonState; // variable to store the last button state
int buttonState;      // variable to store the button state

void setup()
{
  Serial.begin(9600); // initialize serial
  Serial.println("setup started");
  pinMode(BUTTON_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(LED_PIN, OUTPUT);          // set ESP32 pin to output mode

  buttonState = digitalRead(BUTTON_PIN); // read the button state

}
void loop()
{ 
  lastButtonState = buttonState; // store the last button state
  buttonState = digitalRead(BUTTON_PIN); // read the button state
  if (bigloop == 0)
  {
    for (int i = 0; i <= 2; i++)
    {
      Serial.println("loop started " + String(i));
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
      if(i == 2){
        bigloop = 1;
      }
    }
  }
  if(buttonState == LOW && lastButtonState == HIGH){
    Serial.println("button pressed");
    ledstate = !ledstate; // toggle the led state
    digitalWrite(LED_PIN, ledstate); // set the led state
  }
}
