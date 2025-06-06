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

Soldering supplies (optional: directly solder I2C header)

## Software Requirements
##### 1.Arduino IDE (v1.8.13 +)

Must have MPU6050 and Servo libraries installed

##### 2.MPU6050 Library

e.g. Jeff Rowberg’s MPU6050 library

Install via Library Manager or download and place in Arduino/libraries/

##### 3.Servo Library

Built-in in Arduino IDE

##### 4.Wire Library

Built-in (for I2C communication)

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

### How It Works
The core idea is to convert real-time hand orientation (roll, pitch, yaw) from the MPU6050 into corresponding servo angles, then read the flex sensor to open/close the gripper. Below is a step-by-step explanation of each subsystem.

### MPU6050 Orientation Detection
#### 1. I2C Communication Setup

The Arduino’s Wire library initializes I²C at 400 kHz.

MPU6050 is configured for accelerometer ± 2 g and gyroscope ± 250 °/s full-scale ranges.

#### 2. Reading Raw Sensor Data

Accelerometer (AX, AY, AZ) and gyroscope (GX, GY, GZ) readings are retrieved from the MPU6050’s registers at ~100 Hz.
```cpp
// Example: converting raw MPU6050 readings to physical units
float accelX = rawAx / 16384.0;  // in g
float accelY = rawAy / 16384.0;
float accelZ = rawAz / 16384.0;

float gyroX = rawGx / 131.0;  // in °/s
float gyroY = rawGy / 131.0;
float gyroZ = rawGz / 131.0;
```
#### 3. Complementary Filter for Angle Estimation

Accelerometer-based tilt gives reliable long-term reference (prone to noise).

Gyroscope-based angle rate integration gives smooth short-term response (prone to drift).

Complementary filter fuses both:

```cpp
// dt = time elapsed since last reading (in seconds)
float accelPitch = atan2(accelY, accelZ) * RAD_TO_DEG;
float gyroPitchRate = gyroX;  // °/s

pitch = alpha * (pitch + gyroPitchRate * dt) 
      + (1 - alpha) * accelPitch;
```
Similarly for roll:

```cpp
float accelRoll = atan2(-accelX, accelZ) * RAD_TO_DEG;
float gyroRollRate = gyroY;  // °/s

roll = alpha * (roll + gyroRollRate * dt) 
     + (1 - alpha) * accelRoll;
```
Yaw (heading) derived from gyroscope Z only (no magnetometer; accumulates drift, optional).

#### 4. Mapping MPU6050 Angles to Servo Ranges

Base (Yaw) Control: If using gyroscope yaw integration (prone to drift), limit range to ± 90°.
```coo
baseAngle = map(constrain(yaw, -90, +90), -90, +90, 0, 180);
```
Shoulder (Pitch) Control:

pitch ∈ [−45 °, +45 °] (hand tilted up/down)

Map pitch to shoulder servo angle (e.g., 30 ° to 150 °).
```cpp
shoulderAngle = map(constrain(pitch, -45, +45), -45, +45, 150, 30);
```
Wrist (Roll) Control:

roll ∈ [−45 °, +45 °] (hand rolled left/right)

Map roll to wrist servo angle (0 ° to 180 °).

```cpp
wristAngle = map(constrain(roll, -45, +45), -45, +45, 0, 180);
```
### Flex Sensor Gripper Control
#### 1. Voltage Divider Read

Flex sensor and 10 kΩ resistor form a divider; bending finger increases flex resistance (voltage at A0 increases).

Read analogRead(A0) → value ∈ [0, 1023].

#### 2. Thresholding for Open/Close

Define FLEX_THRESHOLD (e.g., 500).

If flexValue > FLEX_THRESHOLD, finger is bent → close gripper (gripperAngle = 180).

Else → open gripper (gripperAngle = 0).

#### 3. Optional Smoothing

Compute running average or apply a simple low-pass filter to prevent jitter when flex value hovers near threshold.

### Servo Mapping & Control
#### 1. Servo Attachments in Code
```cpp
#include <Servo.h>
Servo servoBase, servoShoulder, servoElbow, servoWrist, servoGripper;

const int PIN_BASE      = 3;
const int PIN_SHOULDER  = 5;
const int PIN_ELBOW     = 6;
const int PIN_WRIST     = 9;
const int PIN_GRIPPER   = 10;
```
#### 2. Attaching Servos in setup()
```cpp
void setup() {
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoWrist.attach(PIN_WRIST);
  servoGripper.attach(PIN_GRIPPER);

  // Initialize to neutral positions
  servoBase.write(90);
  servoShoulder.write(90);
  servoElbow.write(90);
  servoWrist.write(90);
  servoGripper.write(0);
}
```
#### 3. Continuous Update Loop
```cpp
void loop() {
  calculateMPU6050Angles();   // Updates global variables: pitch, roll, yaw
  readFlexSensor();           // Updates global variable: flexValue

  // Map sensor values to servo angles:
  int baseAngle      = mapYawToBase(yaw);
  int shoulderAngle  = mapPitchToShoulder(pitch);
  int wristAngle     = mapRollToWrist(roll);
  int elbowAngle     = mapElbowMotion(pitch); // or custom logic
  int gripperAngle   = (flexValue > FLEX_THRESHOLD) ? 180 : 0;

  // Write to servos:
  servoBase.write(constrain(baseAngle, 0, 180));
  servoShoulder.write(constrain(shoulderAngle, 0, 180));
  servoElbow.write(constrain(elbowAngle, 0, 180));
  servoWrist.write(constrain(wristAngle, 0, 180));
  servoGripper.write(constrain(gripperAngle, 0, 180));

  delay(20);  // ~50 Hz update rate
}
```
#### 4. Mapping Functions Example
```cpp
int mapYawToBase(float yaw) {
  // yaw ∈ [-90, +90], map to [0,180]
  return map((int)yaw, -90, +90, 0, 180);
}

int mapPitchToShoulder(float pitch) {
  // pitch ∈ [-45, +45], map to [150, 30] (flip if needed)
  return map((int)pitch, -45, +45, 150, 30);
}

int mapRollToWrist(float roll) {
  // roll ∈ [-45, +45], map to [0,180]
  return map((int)roll, -45, +45, 0, 180);
}

int mapElbowMotion(float pitch) {
  // Example: use pitch > 30° to extend elbow; else flex
  if (pitch > 30) {
    return 150;  // extended
  } else if (pitch < -30) {
    return 30;   // flexed
  } else {
    return 90;   // neutral
  }
}
```
### Customization & Expansion
#### 1. Adjust Sensitivity & Ranges

Tweak alpha in the complementary filter for MPU6050 (e.g., 0.98 for smoother gyro reliance).

Update map() ranges: if your mechanical arm has different joint limits, modify min/max angles accordingly.

Calibrate FLEX_THRESHOLD by printing raw analogRead(A0) values in Serial Monitor while bending.

#### 2. Add Additional Sensors

Integrate a magnetometer (e.g., HMC5883L) with MPU6050 to correct yaw drift and improve base rotation accuracy.

Add a second flex sensor on the middle finger to implement pinch gestures for finer gripper control (variable gripper angles).

#### 3. Swap Mechanical Structure

Replace acrylic parts with 3D-printed PLA or metal brackets for greater load capacity.

Add a 5th DOF (wrist pitch) by mounting a micro-servo at the wrist joint for up/down wrist tilt.

#### 4. Alternate Microcontroller

Port code to an Arduino Mega for more PWM pins and memory for future features.

Use ESP32 (Bluetooth/Wi-Fi) to allow wireless control or remote telemetry.

#### 5. Integrate Feedback & Safety

Add an ultrasonic or limit switch on the base to prevent over-rotation.

Integrate current sensing on servo power lines to detect stall or overload.

Implement Inverse Kinematics

Instead of directly mapping pitch/roll to joint angles, calculate joint angles based on desired end-effector (gripper) position using simple 2D/3D IK solvers.

### Troubleshooting
#### 1. MPU6050 Not Detected

Check I²C wiring: ensure SCL to A5 / SCL, SDA to A4 / SDA, and proper VCC/GND.

In Arduino IDE, open File → Examples → Wire → i2c_scanner to verify MPU6050 I²C address (should respond at 0x68).

If no response, try powering MPU6050 from a separate 5 V source or check solder joints.

#### 2. Unstable or Noisy Angle Readings

Make sure MPU6050 is firmly mounted (no loose connections or vibrations).

Adjust complementary filter coefficient alpha between 0.95 and 0.99 to find optimal balance.

Use low-pass filtering on raw accelerometer/gyro values (e.g., moving average of last 5 readings).

#### 3. Flex Sensor Reads Unchanging Values

Verify flex sensor wiring: one end to 5 V, the other to A0 and to 10 kΩ resistor to GND.

Use Serial Monitor to print analogRead(A0) while bending finger; if values don’t change, check soldering or resistor.

#### 4. Servos Jitter or Stall

Confirm external 5 V supply can source enough current (≥ 2 A).

Add a 100 µF electrolytic capacitor across servo power rails to smooth transients.

Ensure servo GND wires are connected to Arduino GND.

If a servo stalls under load (e.g., heavy gripper payload), consider a higher-torque servo (e.g., MG90S).

#### 5. Arm Moves Erratically or Unexpectedly

Verify sensor calibration: on startup, hold hand level and flat until calibration completes.

Add dead zones: if pitch/roll are near zero (± 2 °), hold servo at neutral to avoid jitter.

Print intermediate sensor values to Serial Monitor to confirm mapping logic.

#### 6. Code Upload Fails

Select the correct board (Uno/Nano/Mega) and processor (for Nano, choose “ATmega328P (Old Bootloader)” if applicable).

Disconnect RX/TX pins (A4/A5 for I²C do not conflict, but if you have soldered additional serial wiring, ensure TX/RX aren’t blocking USB serial).

Ensure no other serial monitor is open when uploading.

### Future Enhancements
#### 1. Wireless Control

Replace USB cable with HC-05 Bluetooth module to stream data to a PC or smartphone.

Build a simple GUI on Processing or Python (PyQt5) to visualize real-time sensor data and allow manual override via keyboard.

#### 2. Voice Control Integration

Use a speech recognition module (e.g., Arduino Voice Recognition Module) to issue basic commands like “home,” “pick,” “place,” which override or complement hand gestures.

#### 3. Advanced Gripper Feedback

Add a force-sensitive resistor (FSR) on the gripper jaws to detect object contact force; stop closing when threshold is reached to avoid crushing.

#### 4. Dual-Arm Coordination

Implement a second MPU6050/flex-sensor set to control a second robotic arm for synchronous or mirrored operations (e.g., assembly tasks).

#### 5. User Calibration Routine

Add a setup mode where the user holds their hand in neutral, max-tilt, and grip positions to automatically calibrate sensor min/max values instead of hardcoding thresholds.

#### 6. Enhanced Mechanical Design

Incorporate bearings or nylon bushings at joints for reduced friction and improved accuracy.

Use an external gearbox for higher torque requirements (e.g., for heavier gripper payloads).

#### 7. Inverse Kinematics (IK) Engine

Implement a 2-link or 3-link IK solver in Arduino (fixed-point math) to compute joint angles from desired end-effector coordinates, using approximate methods.

#### 8. Data Logging & Telemetry

Log MPU6050 and flex sensor readings along with servo commands to an SD card module for post-operation analysis and fine-tuning.

### Contributing
We welcome contributions! If you would like to extend this project, please follow these steps:

#### 1. Fork this repository to your GitHub account.

#### 2. Create a new branch for your feature/bugfix:
```bash
git checkout -b feature/your-feature-name
```
#### 3. Make your changes, test thoroughly, and commit with descriptive messages.

#### 4. Push your branch to your fork:

```bash
git push origin feature/your-feature-name
```
#### 5. Open a Pull Request against the main branch here, describing:

What changes you made

Why they are needed

Any new hardware or software dependencies

#### Before submitting, please ensure:

Code compiles and runs without errors on a fresh Arduino IDE install.

Any added libraries are open-source and included in README_CODE.md or documented.

Mechanical design files (e.g., laser-cut DXF) are added to a new folder if relevant.

For bug reports or feature requests that you do not plan to implement yourself, please open an Issue with as much detail as possible (error messages, wiring photos, Arduino IDE version, etc.).

### License
This project is licensed under the MIT License. See the LICENSE file for full terms and conditions.

#### Enjoy building and customizing your Hand-Motion-Controlled Robotic Arm!
#### Feel free to reach out in Issues for questions, suggestions, or collaboration.
