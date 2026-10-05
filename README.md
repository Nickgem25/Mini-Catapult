# ESP32 Mini Catapult System

## Project Overview
This project is an automated, ESP32-controlled mini catapult designed to wind a launch arm to a precise, user-defined angle before firing. The system relies on a complementary filter combining accelerometer and gyroscope data from an MPU6050/6500 to accurately calculate the arm's tilt in real-time.

## Core Functionality
**The catapult operates through a distinct four-step state machine:**
- **SELECT:** A rotary encoder allows the user to dial in a target launch angle between 0 and 90 degrees. The target angle and current arm angle are displayed on an I2C OLED screen, while a status RGB LED smoothly cycles through different hues.
- **WINDING:** Once the target angle is confirmed by pressing the encoder dial, an MG90S servo engages a dog clutch. A DRV8833-driven motor then winds the catapult arm downwards toward the target. The RGB LED turns solid red during this phase.
- **SETTLING:** To ensure precision, the system pauses to let physical vibrations dampen, then double-checks the sensor reading. It pulses the motor forward or reverse to safely nudge the arm into position until it rests within a 0.3-degree tolerance of the target. The RGB LED turns orange during this calibration.
- **READY:** The RGB LED turns green, indicating the system is successfully aligned. Pressing the dedicated launch button actuates the servo to disengage the clutch, releasing the arm. The system automatically resets back to the SELECT state for the next launch.

## Hardware Components
**The core logic in main.cpp integrates the following hardware:**
- Microcontroller: ESP32   
- Sensors: MPU6050/6500 IMU, Rotary Encoder   
- Actuators: DC Winch Motor (via DRV8833 driver), MG90S Servo (Dog clutch)   
- User Interface: SSD1306 OLED Display, RGB LED, Push Button (Launch)
