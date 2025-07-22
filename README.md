SuperNova 
====
![supernova_final](https://github.com/user-attachments/assets/ad239977-581d-4c70-9d2d-63249994a8e1)

This repository contains the documentation for the SuperNova team's robot for the 2025 World Robot Olympiad Future Engineers competition. the robot, was designed and built by a team of three students.

# Index

- [the Project](#the-project)
- [the Challenge](#the-challenge)
- [Photos of DELTA](#photos-of-delta)
- [Management](#management)
  - [Mobility Management](#mobility-management)
    - Motor DC 12v with encoder
    - Servo Motor 9g
    - Module l298n motor driver
  - [Power and Sense Management](#power-and-sense-management)
    - ESP32 DevKit v1
    - ESP32 expansion board
    - Ultrasonic sensors (hc-sr04)
    - 18650 Battery
    - Pixy Cam v2
    - Voltage regulator
 - [Software/Code Documentation](#softwarecode-documentation--codigodeluxe3_5ino)
   - [1. Overview](#1-overview)
   - [2. Main Components & Libraries](#2-main-components--libraries)
   - [3. Pin Configuration & Hardware Variables](#3-pin-configuration--hardware-variables)
   - [4. Core Variables](#4-core-variables)
   - [5. Main Control Logic](#5-main-control-logic)
   - [6. Key Functions](#6-key-functions)
   - [7. Example: Event Handling Logic](#7-example-event-handling-logic)
   - [8. Usage Instructions](#8-usage-instructions)

the Project
===
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. the vehicle takes different paths depending on the color of the obstacle. We use an Arduino-based program (C++) that includes codes for the ultrasonic sensors (hc-sr04), which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the Arduino circuit board, which then directs the vehicle’s movement system to turn det either clockwise or counterclockwise, as pre-established. Additionally, the vehicle is equipped with a camera module (Pixy v2) to detect colored obstacles and avoid them based on their color

the core of the system is based on the Arduino Mega microcontroller, which provides a versatile platform with sufficient input/output pins and processing capability to manage multiple sensors and actuators for basic robotics applications. the vehicle features a DC motor equipped with an encoder, allowing for precise feedback on wheel rotation to enable closed-loop speed and distance control. Four ultrasonic sensors are strategically mounted to the vehicle, giving comprehensive environmental awareness by measuring distances to obstacles in front and on both sides. A motor driver shield is used to manage power delivery and control signals to the motor, while a servomotor handles steering actuation. the chassis for this vehicle was custom-designed using 3D modeling software and subsequently fabricated with a 3D printer.



## the Challenge
Teams are challenged to create, assemble, and program a robotic car that can drive itself around a racetrack that is dynamically altered for every round. there are two primary objectives in the competition: finishing laps with randomized obstacles and pulling off a flawless parallel parking manoeuvre. Teams must incorporate cutting-edge robotics ideas with an emphasis on innovation and dependability, such as computer vision, sensor fusion, and kinematics.

This challenge emphasizes all aspects of the engineering process, including:

- Mobility Management: Developing efficient vehicle movement mechanisms.
- Obstacle Handling: Strategizing to detect and navigate traffic signs (red and green markers) within specified rules.
- Documentation: Showcasing engineering progress, design decisions, and open-source collaboration through a public GitHub repository.

## Photos of DELTA

| Front view | Back view | Left view | 
| ------------- |------------- | ------------- |
|![DELTA Front (2)](https://github.com/user-attachments/assets/ec1907cf-924b-4a97-a66a-a97e13b13e6b) | ![DELTA Back (2)](https://github.com/user-attachments/assets/1984527a-ac2f-4910-8568-b0b8261cf26b) |![DELTA Left (2)](https://github.com/user-attachments/assets/ab908c82-4331-4858-b076-f0b3fae6ab12) |

| Right view | Top view | Bottom view |
| ------------- | ------------- |------------- |
|![DELTA Right (2)](https://github.com/user-attachments/assets/5a26582e-ae10-4d31-b908-bc338e45cba6) |![DELTA Top (2)](https://github.com/user-attachments/assets/783a9b18-8fbb-4627-ba59-00626b973e42) |![DELTA Bottom (2)](https://github.com/user-attachments/assets/70207e10-3013-434b-9543-bd5aeed7624e) |


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



the propulsion system relies on a DC motor paired with an encoder. the motor provides the mechanical force required to move the vehicle, while the encoder outputs two pulse signals (c!
hannels A and B) that indicate the rotation direction and speed by measuring the number of pulses per revolution.
We used these motors for their balance between power, efficiency, and cost, as well as their easy integration with common motor drivers like the L298N.


**Servo Motor MG995:**
| Specifications: |
| ------------- |
| Voltage: 4.8-6V |
| Torque: 9.4/11 kg/cm |
| Rotation: 180° ± 10°  |
![617BvnN0VJL _SL1500_](https://github.com/user-attachments/assets/b501ca45-12d2-46de-9c1c-de0f9dc0fb90)


A servomotor is integrated into the system to control steering. the servo receives PWM signals from a dedicated Arduino Mega pin, allowing precise angular positioning between 0° and 180°. the servo’s position is controlled programmatically to perform smooth and accurate movements.
We chose this servo for its precise position control, ease of use with Arduino libraries, and widespread availability, offering better accuracy and reliability than cheaper or less-documented servos.

**Module l298n motor driver:**
| Specifications: |
| ------------- |
| Power supply voltage: 5 to 35V  |
| Output current:  0-36mA |
| Can handle:  DC motors or stepper motors |
![modulo-l298n-driver-control-motor-puente-h-arduino-0](https://github.com/user-attachments/assets/50e4565b-47fe-46c2-a331-fc19f75f6467)

the L298N is a robust dual H-bridge motor driver IC capable of controlling two DC motors simultaneously or one stepper motor. Each channel of the L298N can deliver up to 2A continuous current (with adequate heat dissipation) and operates within a voltage range of 5V to 35V, making it suitable for a wide range of motors.
This motor driver module interfaces easily with the ESP32 microcontroller, accepting PWM signals from the ESP32 for precise speed control and digital signals for direction control, enabling both forward and reverse movement. the L298N module also includes built-in protection mechanisms such as current sensing and thermal shutdown to safeguard the system during operation.
the L298N was selected for this project due to its ability to control two motors at once, its compatibility with the ESP32.

### Power and Sense Management

**ESP32 DevKit v1:**

|Specifications:|
| ------------ |
|Microcontroller: ESP32 |
|Flash memory: 4 Mb |
|SRAM: 520 Kb | 
|Frequency: 2.4 GHz to 2.5 GHz |
|Input voltage: 2.7 – 3.6V |
|Pins: 30 |
![ModuloESP32-DEVKITV1-30pines-min_1_2048x2048](https://github.com/user-attachments/assets/efbd9da1-9a60-4602-9d2a-44219e70ab41)

ESP32 DevKit V1:
the ESP32 DevKit V1 is a microcontroller board based on the ESP32 SoC, featuring a dual-core Tensilica processor running at up to 240 MHz. It offers integrated Wi-Fi and Bluetooth, multiple digital input/output pins, analog inputs, PWM outputs, a micro-USB connection, onboard voltage regulator, and reset/boot buttons. the ESP32 board runs the code that enables us to accomplish the challenge, processing sensor data to perform the required movements.

We selected the ESP32 DevKit V1 for its wireless connectivity, higher processing power, and compatibility with a wide range of sensors and actuators. Its user-friendly development environment, strong community support, and extensive documentation make it easier to implement complex robotics tasks compared to less-documented microcontroller boards.


**ESP32 Expansion Base board v1:**

|Specifications:|
| ------------ |
|Power supply: 5-12V|
|On-board Regulator : 5V~1A |
|SRAM: 520 Kb | 
|Frequency: 2.4 GHz to 2.5 GHz |
|Input voltage: 2.7 – 3.6V |
![expansion board](https://github.com/user-attachments/assets/0ba4f354-1d29-45aa-8afe-bd514cc25211)

the ESP32 Expansion Board V1 30P is an add-on board designed to extend the functionality of ESP32 development boards, featuring 30 accessible pins for easy connection to various peripherals. It provides organized access to multiple digital and analog inputs/outputs, power rails, and communication interfaces such as I2C, SPI, and UART. the expansion board simplifies prototyping by offering clearly labeled headers, additional power regulation options, and secure mounting for the ESP32 module, enabling robust connections for sensors, actuators, and external modules.

We selected the ESP32 Expansion Board V1 30P for its convenient pin access, stable power distribution, and compatibility with a broad range of hardware components. Its well-organized layout, ease of integration, and community support streamline the development of complex robotics solutions, making it preferable to less modular expansion boards or direct wiring, which can lead to unreliable connections and increased troubleshooting.


**Ultrasonic Sensor (hc-sr04):**
| Specifications: |
| ------------- |
| Accuracy:  ±3% |
| Measurement Range: 2 a 450 cm |
| Resolution:  0.3 cm |
| Input voltage: 5V  |
| Operating Current: 15mA |
![Ultrasonic_Sensor](https://github.com/user-attachments/assets/ff7ceb5b-2965-475b-acc6-ad80540a37db)


It is a sensor that uses ultrasonic sounds to detect the bounce time of sound from one side to the other. Using the Arduino Mega 2560 we can determine the distance based on the time it takes for the wave to return, performing the function of determining when there is a wall nearby, and thus making the corresponding turn.
This sensor was chosen for its accuracy, low cost, and readily available documentation, outperforming other more expensive or less reliable distance-sensing options.

**18650 Battery:**
| Specifications: |
| ------------- |
| Capacity: 3000 mAh |
| Voltage: 3.7 V |
| Discharge rate:  0.25C |
| Weight: 48 g (one battery) |
| Size: 18 mm diameter, 65 mm length |
![image](https://github.com/user-attachments/assets/09afb713-b1cb-4bf8-b15b-2e0a06950587)

we are going to use 3 of these li-ion batterys to power the vehicle with an 18650 battery holder. This battery configuration provides a reliable and efficient power source for the robot’s electronics and actuators, supporting extended operation during competition or testing.


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

the camera is capable of detecting seven colors simultaneously and It is equipped with an internal processor, which lets us explore just the necessary information for the Arduino to evade in the necessary way, depending on the obstacle colour.
 We selected this camera for its high-resolution image capture, easy interfacing with microcontrollers, and strong community support, providing superior performance and documentation compared to less common camera modules.

**Voltaje Regulator (Lm2596) :**
| Specifications: |
| ------------- |
| Input Voltage: 4V to 35V |
| Output Voltage:  1.23V to 37V |
| Energy eficiency : 80% |
![voltage regulator](https://github.com/user-attachments/assets/7e0a2466-8ae1-4540-9adb-790cd3f9cec8)

The module LM2596 is a regulator step down that can reduce de voltage of input to a lower voltage in output 

# Software/Code Documentation – `CodigoDeluxe3_5.ino`

This document describes the structure, logic, and key functions of the file: `src/CodigoDeluxe3_5.ino` in the WRO-SuperNova project.

---

## 1. Overview

This Arduino C++ program is designed for an autonomous robot and builds upon previous versions by integrating:
- **4 ultrasonic sensors** for obstacle detection (front, rear, left, right)
- **DC motor with encoder** for precise propulsion and distance tracking
- **Servo motor** for steering
- **Adafruit Motor Shield** for motor control
- Optional: Gyroscope/IMU or other advanced sensor modules (if included in this version)

the code uses a state machine (`CarState`) for navigation, decision-making at intersections, straight driving, advanced filtering of sensor data, and dynamic corrections to maintain centering in a corridor. It is likely to use robust filtering for sensor readings and may feature improvements in navigation logic or integration with additional sensors.

---

## 2. Main Components & Libraries

- `AFMotor.h`: Controls Adafruit Motor Shield for DC and servo motors.
- `Servo.h`: Standard Arduino servo control.
- `NewPing.h`: Efficient ultrasonic sensor handling.
- `QuadratureEncoder.h`: Reads encoder pulses for odometry.
- (Optional) `Wire.h`, IMU libraries: For gyroscope/accelerometer integration if present.

---

## 3. Pin Configuration & Hardware Variables

- **Ultrasonic sensors**: Digital pins (TRIGGER and ECHO for each sensor).
- **Encoder**: Analog pins or digital pins as per the actual wiring.
- **Servo**: Digital pin for steering.
- **Motor**: Connected to Motor Shield port as specified in code.
- **IMU (if present)**: Connected via I2C.

---

## 4. Core Variables

- `distanceFront`, `distanceRear`, `distanceLeft`, `distanceRight`: Distance readings from ultrasonic sensors (in cm), likely with outlier rejection/filtering.
- `encoderTicks`, `wheelDiameter`, `wheelCircumference`, `ticksPerRevolution`, `totalDistanceTravelledCm`: For odometry and movement tracking.
- `CarState currentState`: State machine for navigation logic.
- Navigation parameters: Corridor centering, intersection detection, turn decisions.
- Servo angles for left (`servoIzq`), center (`servoCen`), right (`servoDer`) steering.

---

## 5. Main Control Logic

### a. Initialization (`setup()`)

- Sets up serial communication, attaches and centers the servo, stops the motor, resets the encoder, and initializes any additional sensors.
- Sets the initial navigation state, typically to `INICIAL`.

### b. Main Loop (`loop()`)

- Reads ultrasonic and (if present) gyro/IMU sensor data, applying filtering for robustness.
- Updates the encoder-based distance.
- Uses a state machine (`switch(currentState)`) to:
    - Drive straight and keep centered.
    - Detect and handle intersections based on filtered distance readings.
    - Execute left/right turns by comparing open paths.
    - Perform corrections based on lateral sensor data.
    - Optionally, stop or perform special maneuvers as needed.

---

## 6. Key Functions

- **Movement**
    - `avanza(int speed)`: Move forward.
    - `retrocede(int speed)`: Move backward.
    - `detenido()`: Stop.
- **Steering**
    - `girarder()`: Turn right.
    - `girarizq()`: Turn left.
    - `centrado()`: Center the steering.
- **Sensor Reading**
    - `readUltrasonicSensors()`: Reads and filters all distance sensors.
    - Advanced filtering function: Takes multiple samples, discards outliers, calculates robust average.
- **Encoder Management**
    - `updateEncoderDistance()`: Updates total distance using encoder readings.
- **Corrections and Maneuvers**
    - `paredizq()`, `paredder()`: Wall following/turning.
    - `straightDer()`, `straightIzq()`: Small steering corrections to keep centered.
- **Gyroscope/IMU (if present)**
    - Functions to read and use angular data for orientation correction.

---

## 7. Example: Sensor Filtering Logic

```cpp
unsigned int filtrarUltrasonicos(NewPing &sonar) {
    // Take N samples, discard highest/lowest, average the rest for robust reading
}
```

---

## 8. Usage Instructions

1. Open `CodigoDeluxe3_5.ino` in the Arduino IDE.
2. Install all required libraries for motors, servos, ultrasonic sensors, encoder, and (if present) IMU.
3. Connect hardware per configuration in the code.
4. Upload the code to the Arduino.
5. Power on the robot and observe its autonomous behavior.

---

## 9. Notes

- This version likely includes refined filtering for all sensor readings, providing more robust navigation in noisy environments.
- the state machine structure makes it easier to extend or modify navigation behaviors.
- Encoder and (if present) gyroscope integration enable precise movement and orientation correction.
- Update this documentation if the logic or hardware interfaces change.

---







