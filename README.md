# Hand-Motion-Controlled Robotic Arm

This repository contains the code, circuit diagrams, and reference screenshots for a Hand-Motion-Controlled Robotic Arm built using an Arduino, an MPU6050 (gyroscope + accelerometer), and a flex sensor. The system allows intuitive, real-time control of a 4-DOF robotic arm by interpreting hand orientation (MPU6050) and finger bending (flex sensor) to drive arm movements and gripper actions.

---

## Table of Contents

1. [Overview](#overview)  
2. [Key Features](#key-features)  
3. [System Architecture](#system-architecture)  
4. [Hardware Components](#hardware-components)  
5. [Software Requirements](#software-requirements)  
6. [Repository Structure](#repository-structure)  
7. [Circuit Diagram](#circuit-diagram)  
8. [Setup & Installation](#setup--installation)  
   1. [Hardware Assembly](#hardware-assembly)  
   2. [Ardu​ino IDE & Libraries](#arduino-ide--libraries)  
   3. [Wiring Connections](#wiring-connections)  
   4. [Loading the Code](#loading-the-code)  
   5. [Powering the System](#powering-the-system)  
9. [How It Works](#how-it-works)  
   1. [MPU6050 Orientation Detection](#mpu6050-orientation-detection)  
   2. [Flex Sensor Gripper Control](#flex-sensor-gripper-control)  
   3. [Servo Mapping & Control](#servo-mapping--control)  
10. [Customization & Expansion](#customization--expansion)  
11. [Troubleshooting](#troubleshooting)  
12. [Future Enhancements](#future-enhancements)  
13. [Contributing](#contributing)  
14. [License](#license)  

---

## Overview

The Hand-Motion-Controlled Robotic Arm leverages an MPU6050 (gyroscope + accelerometer) strapped to the user’s hand to capture hand orientation (pitch, roll, yaw) and a flex sensor on the index finger to detect bending for gripper control. Those sensor readings are processed by an Arduino, which then drives four SG90 micro-servos (base rotation, shoulder, elbow, wrist) for spatial movement and a fifth servo for gripper open/close. The result is a near-real-time, intuitive interface: tilt or rotate your hand to move the arm and bend your finger to close/open the gripper.

---

## Key Features

- **Gesture-Based 4-DOF Movement**  
  - **Base Rotation (Yaw):** Hand rotation around the vertical axis (left/right twist)  
  - **Shoulder (Pitch):** Forward/backward hand tilt controls up/down movement  
  - **Elbow (Extension/Flexion):** Hand tilts forward/back beyond threshold to extend/flex elbow  
  - **Wrist (Roll):** Lateral hand roll (left/right tilt) controls wrist rotation  

- **Gripper Control via Flex Sensor**  
  - Bending the index finger (flex sensor) closes the gripper; straightening opens it  

- **Real-Time Responsiveness**  
  - MPU6050 sampling at 100 Hz (I²C) and flex sensor analog read at 10 bit ADC (≈100 Hz)  
  - Near-instantaneous servo updates for smooth arm motion  

- **Modular & Customizable Code**  
  - Each sensor reading and servo mapping handled in separate functions  
  - Easy to adjust sensitivity thresholds, movement ranges, or calibrate sensor offsets  

- **Comprehensive Documentation**  
  - Detailed circuit schematics, wiring diagrams, and reference screenshots included  
  - Step-by-step setup instructions and troubleshooting tips  

---

## System Architecture

```plaintext
+──────────────────────────────────────────+
|          User’s Hand (Wearable)         |
| ┌─────────────────────────────────────┐  |
| │  MPU6050 (I²C: SCL, SDA, VCC, GND) │  |
| │  Flex Sensor (Analog Input to A0)  │  |
| └─────────────────────────────────────┘  |
+──────────────────────────────────────────+
                    │ (I²C + Analog)
                    ↓
+──────────────────────────────────────────+
|             Arduino Uno/Nano            |
| ┌─────────────────────────────────────┐  |
| │  Read MPU6050 (pitch, roll, yaw)   │  |
| │  Read Flex Sensor (0–1023 ADC)     │  |
| │  Calculate Servo Angles (0–180°)   │  |
| │  Control 5 × SG90 Servos via PWM   │  |
| └─────────────────────────────────────┘  |
+──────────────────────────────────────────+
                    │ (PWM Signals + 5 V Power)
                    ↓
+──────────────────────────────────────────+
|   Robotic Arm (4 DOF + Gripper)         |
| ┌─────────────────────────────────────┐  |
| │  Servo 1 (Pin 3): Base Rotation     │  |
| │  Servo 2 (Pin 5): Shoulder Up/Down  │  |
| │  Servo 3 (Pin 6): Elbow Extend/Flex │  |
| │  Servo 4 (Pin 9): Wrist Left/Right  │  |
| │  Servo 5 (Pin 10): Gripper Open/Close │ │
| └─────────────────────────────────────┘  |
+──────────────────────────────────────────+
```

## Hardware Components
Arduino Board

Uno, Nano, or Mega (5 V logic). In examples, Arduino Uno is used.

MPU6050 Sensor

6-axis gyroscope + accelerometer (I²C interface)

Flex Sensor

Analog bending sensor (resistance varies when finger bends)

Servo Motors × 5

SG90 micro-servos (1.8 kg·cm torque recommended)

### Assignments:

Servo 1 (Pin 3): Base Rotation (forward/reverse movement)

Servo 2 (Pin 5): Shoulder (up/down movement)

Servo 3 (Pin 6): Gripper (open/close)

Servo 4 (Pin 9): Left/Right (wrist rotation)

Servo 5 (Pin 10): Elbow (extension/flexion)

### Power Supply

5 V DC regulated (capable of ≥ 2 A)

Common ground between Arduino and servos

Breadboard & Jumper Wires

For prototyping connections

Hand-Mounted Setup

Velcro straps or 3D-printed mount to secure MPU6050 & flex sensor to user’s hand

### Miscellaneous

Nylon spacers, M3 screws for connecting acrylic/metal parts (if custom arm chassis)

Soldering supplies (optional: directly solder I²C header)

## Software Requirements
Arduino IDE (v1.8.13 +)

Must have MPU6050 and Servo libraries installed

MPU6050 Library

e.g. Jeff Rowberg’s MPU6050 library

Install via Library Manager or download and place in Arduino/libraries/

Servo Library

Built-in in Arduino IDE

Wire Library

Built-in (for I²C communication)

Optional Serial Monitor/Plotter

For debugging raw sensor values (Arduino Serial Plotter)



