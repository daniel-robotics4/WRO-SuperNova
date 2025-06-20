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
 - [Software/Code Documentation](#softwarecode-documentation--codigodeluxe1_9ino)
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
|![DELTA front](https://github.com/user-attachments/assets/37a291b3-2074-4cc4-9628-6d25076077c3)|![DELTA back](https://github.com/user-attachments/assets/f7437120-5ef7-4e59-80dd-d65c5fc95d8d)|![delta left](https://github.com/user-attachments/assets/6ff604bb-370f-42d3-ae5e-385f7e50b24d) |

| Right view | Top view | Bottom view |
| ------------- | ------------- |------------- |
|![DELTA right](https://github.com/user-attachments/assets/df51e0d2-e79f-4c43-a7e4-7354724a193e)|![DELTA top](https://github.com/user-attachments/assets/9fcd98e8-019f-4f3a-8752-4e90184c991b)|![DELTA bottom](https://github.com/user-attachments/assets/6b381bab-34fb-445d-84d1-24efc51fd2fd)|


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




# Software/Code Documentation – `CodigoDeluxe2_0.ino`

This document describes the structure, logic, and key functions of the file: `src/CodigoDeluxe2_0.ino` in the WRO-SuperNova project.

---

## 1. Overview

This Arduino C++ program controls an autonomous robot using:
- **4 ultrasonic sensors** for obstacle detection (front, rear, left, right)
- **DC motor with encoder** for propulsion and distance measurement
- **Servo motor** for steering
- **Adafruit Motor Shield** for motor control

The code uses a state machine (`CarState`) to manage navigation behaviors, including straight navigation, stopping, decision-making at intersections, and executing left/right turns, as well as dynamic corrections to maintain centering in a corridor. It features a `sentido` variable to remember the last chosen direction at intersections.

---

## 2. Main Components & Libraries

- `AFMotor.h`: Controls the Adafruit Motor Shield for DC and servo motors.
- `Servo.h`: Standard Arduino servo control.
- `NewPing.h`: Efficient handling of ultrasonic sensors.
- `QuadratureEncoder.h`: Reads pulses from the drive wheel encoder.

---

## 3. Pin Configuration & Hardware Variables

- **Ultrasonic sensors**: Digital pins (TRIGGER and ECHO for each sensor).
- **Encoder**: Analog pins A8, A9.
- **Servo**: Digital pin 38.
- **Motor**: Connected to Motor Shield port M3.

---

## 4. Core Variables

- `distanceFront`, `distanceRear`, `distanceLeft`, `distanceRight`: Distance readings from ultrasonic sensors (in cm).
- `encoderTicks`, `wheelDiameter`, `wheelCircumference`, `ticksPerRevolution`, `totalDistanceTravelledCm`: For measuring and calculating distance traveled.
- `CarState currentState`: Controls the robot's mode (e.g., INICIAL, STRAIGHT, DECIDIR_SENTIDO, GIROLEFT, GIRORIGHT, SENTIDO, STOPPED, etc.).
- Navigation parameters: `objetoDelante`, `paredCerca`, `paredLejos`, `carritoCentrado`, `toleranciaPared`, `pasilloAbierto`, `pasilloCerrado` for flexible environmental adaptation.
- `servoIzq`, `servoCen`, `servoDer`: Positions (angles) for left, center, and right steering.
- `sentido`: Remembers the last chosen direction at intersections (1 = right, 2 = left).

---

## 5. Main Control Logic

### a. Initialization (`setup()`)

- Initializes serial communication for debugging.
- Attaches the servo and sets it to the center position.
- Stops the motor and resets encoder counters.
- Initial state is `INICIAL`.

### b. Main Loop (`loop()`)

- Reads all ultrasonic sensor values and updates encoder distance.
- Prints current encoder and state for debugging.
- State machine (`switch(currentState)`) handles robot modes:
    - **INICIAL**: Sets up, moves forward, transitions to `DECIDIR_SENTIDO` if an obstacle is detected ahead.
    - **STRAIGHT**: Moves forward, keeps centered in the corridor using lateral corrections, and transitions to `SENTIDO` if an obstacle is detected ahead and a direction has already been chosen.
    - **DECIDIR_SENTIDO**: Decides whether to turn left or right at an intersection based on open paths, sets `sentido`, and transitions to the appropriate turn state.
    - **GIROLEFT/GIRORIGHT**: Executes left or right turn sequence, then transitions to `STRAIGHT`.
    - **SENTIDO**: If `sentido` is set, transitions to the corresponding turn state.
    - **STOPPED**: Stops the robot.
    - **Default**: Stops the robot.

### c. Correction and Maneuver Logic

- **Dynamic centering**: If robot is too close or too far from the left/right wall, makes small corrections (via `straightDer`, `straightIzq`) to re-center.
- **Decision-making**: At intersections, checks which direction is open and executes the appropriate turn sequence, updating `sentido`.

---

## 6. Key Functions

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
    - `paredizq()`, `paredder()`: Wall following/turning.
    - `straightDer()`, `straightIzq()`: Small corrections to maintain centering.

---

## 7. Example: State Machine and Correction Logic

```cpp
switch (currentState) {
  case STRAIGHT:
    avanza(170);
    if (distanceFront < objetoDelante && distanceFront != MAX_DISTANCE && sentido != 0) {
      currentState = SENTIDO;
      break;
    }
    if (distanceRight != MAX_DISTANCE && distanceLeft != MAX_DISTANCE) {
      if (distanceLeft < paredCerca - toleranciaPared) {
        straightDer();
      } else if (distanceRight < paredCerca - toleranciaPared && distanceRight < distanceLeft) {
        straightIzq();
      } else {
        centrado();
      }
    } else {
      centrado();
    }
    break;
}
```

---

## 8. Usage Instructions

1. Open `CodigoDeluxe2_0.ino` in the Arduino IDE.
2. Install required libraries (AFMotor, Servo, NewPing, QuadratureEncoder).
3. Connect hardware per the pin configuration.
4. Upload the code to the Arduino Mega 2560.
5. Power on the robot and observe autonomous operation.

---











