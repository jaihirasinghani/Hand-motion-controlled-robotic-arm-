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
##### 1. Arduino Board

Uno, Nano, or Mega (5 V logic). In examples, Arduino Uno is used.

##### 2. MPU6050 Sensor

6-axis gyroscope + accelerometer (I²C interface)

##### 3. Flex Sensor

Analog bending sensor (resistance varies when finger bends)

##### 4. Servo Motors × 5

SG90 micro-servos (1.8 kg·cm torque recommended)

Assignments:

Servo 1 (Pin 3): Base Rotation (forward/reverse movement)

Servo 2 (Pin 5): Shoulder (up/down movement)

Servo 3 (Pin 6): Gripper (open/close)

Servo 4 (Pin 9): Left/Right (wrist rotation)

Servo 5 (Pin 10): Elbow (extension/flexion)

##### 5. Power Supply

5 V DC regulated (capable of ≥ 2 A)

Common ground between Arduino and servos

##### 6. Breadboard & Jumper Wires

For prototyping connections

##### 7.Hand-Mounted Setup

Velcro straps or 3D-printed mount to secure MPU6050 & flex sensor to user’s hand

##### 8. Miscellaneous

Nylon spacers, M3 screws for connecting acrylic/metal parts (if custom arm chassis)

Soldering supplies (optional: directly solder I²C header)

## Software Requirements
##### 1.Arduino IDE (v1.8.13 +)

Must have MPU6050 and Servo libraries installed

##### 2.MPU6050 Library

e.g. Jeff Rowberg’s MPU6050 library

Install via Library Manager or download and place in Arduino/libraries/

##### 3.Servo Library

Built-in in Arduino IDE

##### 4.Wire Library

Built-in (for I²C communication)

##### 5.Optional Serial Monitor/Plotter

For debugging raw sensor values (Arduino Serial Plotter)

## Circuit Diagram
All wiring schematics are in the Circuit_Diagram/ folder. Below is a summary of key connections:

#### 1. MPU6050 ↔ Arduino

MPU6050 VCC → Arduino 5 V

MPU6050 GND → Arduino GND

MPU6050 SCL → Arduino A5 (Uno) / D21 (Mega)

MPU6050 SDA → Arduino A4 (Uno) / D20 (Mega)

#### 2. Flex Sensor ↔ Arduino

Flex Sensor (one end) → 5 V

Flex Sensor (other end) → Analog Input A0 + 10 kΩ pull-down to GND

When the finger bends, resistance increases → analog read value changes

#### 3. Servos ↔ Arduino & Power

Servo Grounds (all) → Common GND (Arduino GND)

Servo VCC (all) → External 5 V supply

Ensure common ground between Arduino and servo power rail

Servo Signal → Arduino PWM Pins

Servo 1 (Base): Pin 3

Servo 2 (Shoulder): Pin 5

Servo 3 (Gripper): Pin 6

Servo 4 (Wrist Left/Right): Pin 9

Servo 5 (Elbow): Pin 10 (or any free PWM pin)

#### 4. Power Supply

Use a 5 V regulated power supply rated ≥ 2 A

Tie its GND to Arduino GND

Do NOT power servos from Arduino 5 V regulator (limited current)

Refer to Circuit_Diagram/Full_Schematic.pdf for the complete circuit overview.

## Setup & Installation
### Hardware Assembly
#### 1. Mount the MPU6050 & Flex Sensor to Hand

Secure MPU6050 breakout board on top of user’s hand (dorsal side) using Velcro or a 3D-printed holder.

Attach flex sensor along the index finger such that bending causes measurable resistance change.

Route sensor wires to Arduino mounted on wrist or backpack.

#### 2. Build the Robotic Arm Chassis

Assemble the 4-DOF arm using acrylic sheets or laser-cut parts (base, shoulder, elbow, wrist joint, and gripper).

Attach each SG90 servo horn to its corresponding joint with M2.5 screws & spacers.

Mount Arduino Uno on base plate or an independent board with standoffs.

#### 3. Wire All Components on a Breadboard

Place Arduino on breadboard or mount directly.

Connect MPU6050 (I²C) wiring and flex sensor (A0) according to schematics.

Route servo power leads to the external 5 V supply & signal wires to Arduino PWM pins.

Double-check all GNDs are common (Arduino, sensors, servos, power supply).

### Arduino IDE & Libraries
#### 1. Install Arduino IDE

Download from arduino.cc and install.

#### 2. Install MPU6050 Library

Open Arduino IDE → Sketch → Include Library → Manage Libraries…

Search for “MPU6050” (e.g. by Jeff Rowberg) and install it.

#### 3. Verify Servo & Wire Libraries

The Servo and Wire libraries are bundled with the Arduino IDE; no extra installation needed.

### Wiring Connections
#### 1. MPU6050 ↔ Arduino

Connect as follows (Uno pins shown):

MPU6050 VCC → Arduino 5 V

MPU6050 GND → Arduino GND

MPU6050 SCL → Arduino A5

MPU6050 SDA → Arduino A4

#### 2. Flex Sensor ↔ Arduino

One end of flex sensor → Arduino 5 V

The other end → Arduino A0 and to a 10 kΩ resistor which is tied to GND

This forms a voltage divider: as the flex sensor bends, voltage at A0 changes.

#### 3. Servos ↔ Arduino & Power

External 5 V supply → Servo VCC leads (red wires)

All servos GND (brown wires) → External 5 V supply GND & Arduino GND

Servo signals (yellow/orange wires) → Arduino PWM pins:

Pin 3 → Servo 1 (Base)

Pin 5 → Servo 2 (Shoulder)

Pin 6 → Servo 3 (Gripper)

Pin 9 → Servo 4 (Wrist)

Pin 10 → Servo 5 (Elbow)

#### 4. Power Supply

Ensure the external 5 V GND is tied to Arduino GND.

Do NOT draw servo power from Arduino’s 5 V pin.

### Loading the Code
#### 1. Open Arduino Sketch

In Arduino IDE: File → Open, navigate to Code/HandGestureArm.ino.

#### 2. Select the Board & Port

Tools → Board → Arduino Uno (or Nano/Mega if using a different board)

Tools → Port → COMx (Windows) or /dev/ttyUSBx (Linux/macOS)

#### 3. Verify & Upload

Click the Verify button (✓) to compile.

Click Upload (→) to flash the code onto the Arduino.

#### 4. Open Serial Monitor (Optional)

Tools → Serial Monitor, set baud to 115200.

View raw sensor values for calibration or debugging.

### Powering the System
#### 1. Power Android Board

Insert USB cable to power Arduino from a PC or 5 V adapter.

#### 2. Power Servos & Sensors

Connect the external 5 V supply to servo VCC

Confirm all grounds are common.

#### 3. Initial Calibration

Upon power-up, the code reads baseline MPU6050 offsets; hold your hand in the neutral (flat) position (wrist horizontal, palm facing down) until calibration LED (if available) stops blinking.

The flex sensor at rest (finger straight) should read ≈ 300–400 (ADC units); adjust FLEX_THRESHOLD in code if needed.



