# Automatic Gate Demo Kit 🚧🔐

This project demonstrates an automatic gate control system using an **Arduino Uno**, designed to respond to nearby objects using an **ultrasonic sensor** and control hardware components like a **servo motor**, **LEDs**, and a **buzzer**.

## 🔧 Features

- **Ultrasonic Sensor**: Detects objects within a 10 cm range.
- **Servo Motor**: Opens the gate (rotates to 90°) when an object is detected, and closes it after 5 seconds of no detection.
- **Red LED**: Indicates the gate is closed.
- **Blue LED**: Indicates the gate is open.
- **Buzzer**: Beeps continuously while the gate is open.

## 📁 Files

- `automatic_gate.ino` — Main Arduino source code for controlling the gate.
- `README.md` — This file, explaining the project setup and behavior.

## 🧠 How It Works

1. The ultrasonic sensor constantly checks for an object within the threshold distance (10 cm).
2. If an object is detected:
   - The gate opens (servo rotates to 90°).
   - The red LED turns off, blue LED turns on.
   - The buzzer starts beeping.
3. If no object is detected for 5 seconds:
   - The gate closes (servo returns to 0°).
   - The red LED turns on, blue LED turns off.
   - The buzzer stops.

## 🛠️ Hardware Requirements

- Arduino Uno
- Ultrasonic Sensor (HC-SR04)
- Servo Motor
- Red LED with current-limiting resistor
- Blue LED with current-limiting resistor
- Buzzer
- Jumper wires, Breadboard, and Power source

