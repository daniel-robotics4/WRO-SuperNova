SuperNova 
====
![supernova_final](https://github.com/user-attachments/assets/ad239977-581d-4c70-9d2d-63249994a8e1)

This repository contains the documentation for the SuperNova team's robot for the 2025 World Robot Olympiad Future Engineers competition. The robot, was designed and built by a team of three students.

# Index

- [The Project](#the-project)
- [The Challenge](#the-challenge)
- [Photos of DELTA](#photos-of-delta)
- [Management](#management)
  - [Mobility Management](#mobility-management)
    - Motor DC 12v with encoder
    - Servo Motor 9g
    - Motor driver shield (L293)
  - [Power and Sense Management](#power-and-sense-management)
    - Arduino Mega 2560
    - Ultrasonic sensors (HC-SR04)
    - 18650 Battery
    - Pixy Cam v2
 - [Software/Code Documentation](#softwarecode-documentation--codigodeluxe1_6ino)
   - [1. Overview](#1-overview)
   - [2. Main Components & Libraries](#2-main-components--libraries)
   - [3. Pin Configuration & Hardware Variables](#3-pin-configuration--hardware-variables)
   - [4. Core Variables](#4-core-variables)
   - [5. Main Control Logic](#5-main-control-logic)
   - [6. Key Functions](#6-key-functions)
   - [7. Example: Event Handling Logic](#7-example-event-handling-logic)
   - [8. Usage Instructions](#8-usage-instructions)

The Project
===
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. The vehicle takes different paths depending on the color of the obstacle. We use an Arduino-based program (C++) that includes code for ultrasonic sensors, which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the Arduino circuit board, which then directs the vehicle’s movement system to turn either clockwise or counterclockwise, as pre-established. Additionally, the vehicle is equipped with a camera module (Pixy v2) to detect colored obstacles and avoid them based on their color

The core of the system is based on the Arduino Mega 2560 microcontroller, which offers ample input/output pins and processing power to handle multiple sensors and actuators simultaneously. The vehicle integrates a DC motor equipped with an encoder to provide precise feedback on wheel rotation, enabling closed-loop speed and distance control. Three ultrasonic sensors are strategically mounted to provide comprehensive environmental awareness by measuring distances to obstacles in front and on both sides of the vehicle. A motor driver shield manages power delivery and control signals to the motor, while a servomotor is used to actuate steering,the chasis made for this vehicle was made from scratch in a 3d tool an then printed

## The Challenge
Teams are challenged to create, assemble, and program a robotic car that can drive itself around a racetrack that is dynamically altered for every round. There are two primary objectives in the competition: finishing laps with randomized obstacles and pulling off a flawless parallel parking manoeuvre. Teams must incorporate cutting-edge robotics ideas with an emphasis on innovation and dependability, such as computer vision, sensor fusion, and kinematics.

This challenge emphasizes all aspects of the engineering process, including:

- Mobility Management: Developing efficient vehicle movement mechanisms.
- Obstacle Handling: Strategizing to detect and navigate traffic signs (red and green markers) within specified rules.
- Documentation: Showcasing engineering progress, design decisions, and open-source collaboration through a public GitHub repository.

## Photos of DELTA

| Front view | Back view | Left view | 
| ------------- |------------- | ------------- |
|![DSC00016](https://github.com/user-attachments/assets/cbd4d9f3-fd41-434f-a217-3c01315bc188)| ![DSC00021](https://github.com/user-attachments/assets/e64a4ca4-93cb-49c7-bc41-8eb714ad8953)| ![DSC00017](https://github.com/user-attachments/assets/98e1e0f9-2ff2-47d7-bc34-071b20b42f02)|

| Right view | Top view | Bottom view |
| ------------- | ------------- |------------- |
|![DSC00015](https://github.com/user-attachments/assets/72e12995-067b-44ab-b539-f2c24c23a5f6)| ![DSC00019](https://github.com/user-attachments/assets/1f0e0ec6-7680-4ed6-98db-8d491bfb30c5) | ![DSC00020](https://github.com/user-attachments/assets/9dfdb270-9df1-44d0-a644-f70bf5376c24)|

## Management

### Mobility Management

**Motor DC 12v with encoder:**
| Specifications: |
| ------------- |
| Voltage: 12V |
| Gear Ratio: 1:34 |
| Speed (at 12V): 126 RPM |
| Torque: 4.2 kg·cm |
| Weight: 86g |
| Encoder: Optical, 12 counts per revolution |
![encoder2](https://github.com/user-attachments/assets/47492287-88c9-4dba-ba15-793ce49024c4)



The propulsion system relies on a DC motor paired with an encoder. The motor provides the mechanical force required to move the vehicle, while the encoder outputs two pulse signals (c!
hannels A and B) that indicate the rotation direction and speed by measuring the number of pulses per revolution.

**Servo Motor 9g:**
| Specifications: |
| ------------- |
| Voltage: 4.8-6V |
| Torque: 1.6-1.8 kg/cm |
| Rotation: 180° ± 10°  |
![servo](https://github.com/user-attachments/assets/616dea0b-ce80-44be-adf4-3006b407275c)

A servomotor is integrated into the system to control steering. The servo receives PWM signals from a dedicated Arduino Mega pin, allowing precise angular positioning between 0° and 180°. The servo’s position is controlled programmatically to perform smooth and accurate movements.

**Motor driver shield (L293):**
| Specifications: |
| ------------- |
| Power supply voltage: 4.5 to 25V  |
| Output current: 1.2A (peak) |
| Can handle: 4 DC motors or 2 stepper motors and 2 servo motors |
![mtr cntrlr](https://github.com/user-attachments/assets/fd5479d2-4d84-43ed-bd1c-b6e060d37bcf)

It is a driver board based on L293 IC, which can drive 4 DC motors and 2 stepper or Servo motors at the same time. Each channel of this module has the maximum current of 1.2A and doesn't work if the voltage is more than 25v or less than 4.5v.
The motor driver shield is designed to handle the power requirements of the DC motor and provide an easy interface for Arduino Mega. It accepts PWM signals for speed control and digital signals for motor direction, enabling forward and reverse movement. The shield includes built-in protection features such as current limiting and thermal shutdown to safeguard components during operation.

### Power and Sense Management
**Arduino Mega 2560:**
| Specifications: |
| ------------- |
| Microcontroller: ATmega2560 | 
| Flash memory: 256 kB |
| SRAM: 8 kB |
| Frequency: 16 MHz |
| Pins: 56 |
| Input voltage: 5V  |
 ![mega 2](https://github.com/user-attachments/assets/795966a0-d3ef-4f3b-9485-fe8bbaa3a449)
 
Arduino Mega 2560: Is a microcontroller board based on the ATmega2560. It has 54 digital input/output pins and 16 analog inputs, a 16 MHz crystal oscillator, a USB connection, a power jack, an ICSP header, and a reset button. The arduino is the board that contains the code that allow us to accomplish the challenge, using the sensors data to make the necessary movements.

**Ultrasonic sensors (hc-sr04):**
| Specifications: |
| ------------- |
| Accuracy: 3cm |
| Measurement Range: 2cm to 400cm |
| Resolution: 0.3 cm |
| Frequency: 40 kHz|
| Input voltage: 5V  |
| Operating Current: 15 mA |
![Ultrasonic_Sensor](https://github.com/user-attachments/assets/9172b512-f920-4b1e-98cb-fac84d70ee8a)


It is a sensor that uses ultrasonic sounds to detect the bounce time of sound from one side to the other. Using the Arduino Mega 2560 we can determine the distance based on the time it takes for the wave to return, performing the function of determining when there is a wall nearby, and thus making the corresponding turn.


**18650 Battery:**
| Specifications: |
| ------------- |
| Capacity: 3000 mAh |
| Voltage: 3.7 V |
| Discharge rate:  0.25C |
| Weight: 48 g (one battery) |
| Size: 18 mm diameter, 65 mm length |
![image](https://github.com/user-attachments/assets/09afb713-b1cb-4bf8-b15b-2e0a06950587)

we are going to use 3 of these li-ion batterys to power the vehicle with an 18650 battery holder.


**PixyCam v2:**
| Specifications: |
| ------------- |
| Processor: NXP LPC4330 |
| Image sensor: Aptina MT9M114 |
| Resolution:1296×976  |
| Lens field-of-view: 80° horizontal, 40° vertical |
| Power consumption: 140mA |
| RAM: 264K bytes |
| Flash Memory: 2M bytes |
| Power input: 5V |
![pixy-v21-camera-sensor](https://github.com/user-attachments/assets/74f57132-97c9-4abd-84b3-dc63150acd27)

The camera is capable of detecting seven colors simultaneously and It is equipped with an internal processor, which lets us explore just the necessary information for the Arduino to evade in the necessary way, depending on the obstacle colour.


## Software/Code Documentation – `CodigoDeluxe1_7.ino`

This document describes the structure, logic, and key functions of the file: `src/codigoDeluxe1_7.ino` in the WRO-SuperNova project.

---

### 1. Overview

This Arduino C++ program controls an autonomous robot using:
- **3 ultrasonic sensors** for obstacle detection
- **DC motor with encoder** for movement and distance measurement
- **Servo motor** for steering
- **Motor Shield** for motor control
- **Timer interrupt** for periodic sensor/event processing

Sensor readings and state logic are used for real-time autonomous navigation and obstacle avoidance.

---

### 2. Main Components & Libraries

- `AFMotor.h`: Controls the Adafruit Motor Shield for DC and servo motors.
- `Servo.h`: Standard Arduino servo control.
- `NewPing.h`: Efficient handling of ultrasonic sensors.
- `QuadratureEncoder.h`: For reading the encoder pulses on the drive wheel.
- `TimerOne.h`: For timer interrupts and periodic function execution.

---

### 3. Pin Configuration & Hardware Variables

- **Ultrasonic sensors**: Digital pins (TRIGGER and ECHO for each sensor).
- **Encoder**: Analog pins A8, A9.
- **Servo**: Digital pin 38 (`SERVO_PIN`).
- **Motor**: Connected to Motor Shield port M3.

---

### 4. Core Variables

- `distanceFront`, `distanceRear`, `distanceLeft`, `distanceRight`: Distance readings from ultrasonic sensors (in cm).
- `encoderTicks`, `wheelDiameter`, `wheelCircumference`, `ticksPerRevolution`, `totalDistanceTravelledCm`: For measuring and calculating distance traveled.
- `servoIzq`, `servoCen`, `servoDer`: Positions (angles) for left, center, and right steering.
- `Switch`, `bandera`: State/mode variables for maneuver selection and execution.

---

### 5. Main Control Logic

#### a. Initialization (`setup()`)

- Initializes serial communication for debugging.
- Attaches the servo and sets it to the center position.
- Stops the motor and resets encoder counters.
- Sets up a timer interrupt using `Timer1` to call `handleUltrasonicEvents()` every 100 ms for responsive sensor-based decisions.

#### b. Main Loop (`loop()`)

- Reads all ultrasonic sensor values.
- Prints sensor and encoder readings for debugging.
- Moves forward by default (`avanza(200)`).

#### c. Timer-Based Event Handling (`handleUltrasonicEvents()`)

- Reads ultrasonic sensors and evaluates conditions for maneuvers:
    - If right is open and front blocked: turn right.
    - If left is open and front blocked: turn left.
    - If right or left is blocked: perform correction.
    - If front is blocked (with last direction): perform reverse and correction (bandera 1 or 2).
    - Otherwise: move forward.
- Calls the appropriate maneuver and updates encoder distance.

---

### 6. Key Functions

- **Movement**
    - `avanza(int speed)`: Move forward.
    - `retrocede(int speed)`: Move backward.
    - `detenido()`: Stop.
- **Steering**
    - `girarder()`: Turn servo right.
    - `girarizq()`: Turn servo left.
    - `centrado()`: Center the steering.
- **Sensor Reading**
    - `readUltrasonicSensors()`: Acquire all sensor data and handle out-of-range readings.
- **Encoder Management**
    - `updateEncoderDistance()`: Calculate and store distance traveled.
- **Corrections and Maneuvers**
    - `paredizq()`, `paredder()`: Wall following/turning with distance update.
    - `correccionFrontalBandera1/2()`: Correction after a frontal obstacle.
    - `correccionDerecha()`, `correccionIzquierda()`: Side corrections.

---

## #7. Example: Event Handling Logic

```
void handleUltrasonicEvents() {
    readUltrasonicSensors();

    if ((distanceRight > 100) && (distanceFront < 10)){
        paredder(); return;
    }
    if ((distanceLeft > 100) && (distanceFront < 10)){
        paredizq(); return;
    }
    if (distanceRight < 5) {
        correccionIzquierda(); return;
    }
    if (distanceLeft < 5) {
        correccionDerecha(); return;
    }
    if ((distanceFront < 10) && bandera == 1) {
        correccionFrontalBandera1(); return;
    }
    if ((distanceFront < 10) && bandera == 2) {
        correccionFrontalBandera2(); return;
    }
    avanza(100); // Default action
}
```

---

### 8. Usage Instructions

1. Open `CodigoDeluxe1_6.ino` in the Arduino IDE.
2. Install required libraries (AFMotor, Servo, NewPing, QuadratureEncoder, TimerOne).
3. Connect hardware per the pin configuration.
4. Upload the code to the Arduino Mega 2560.
5. Power on the robot and observe autonomous operation.

---







