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
    - Ultrasonic sensors (hc-sr04)
    - 18650 Battery
    - Pixy Cam v2
 - [Software/Code Documentation](#softwarecode-documentation--codigodeluxe3_5ino)
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
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. The vehicle takes different paths depending on the color of the obstacle. We use an Arduino-based program (C++) that includes codes for the ultrasonic sensors (hc-sr04), which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the Arduino circuit board, which then directs the vehicle’s movement system to turn det either clockwise or counterclockwise, as pre-established. Additionally, the vehicle is equipped with a camera module (Pixy v2) to detect colored obstacles and avoid them based on their color

The core of the system is based on the Arduino Mega microcontroller, which provides a versatile platform with sufficient input/output pins and processing capability to manage multiple sensors and actuators for basic robotics applications. The vehicle features a DC motor equipped with an encoder, allowing for precise feedback on wheel rotation to enable closed-loop speed and distance control. Four ultrasonic sensors are strategically mounted to the vehicle, giving comprehensive environmental awareness by measuring distances to obstacles in front and on both sides. A motor driver shield is used to manage power delivery and control signals to the motor, while a servomotor handles steering actuation. The chassis for this vehicle was custom-designed using 3D modeling software and subsequently fabricated with a 3D printer.



## The Challenge
Teams are challenged to create, assemble, and program a robotic car that can drive itself around a racetrack that is dynamically altered for every round. There are two primary objectives in the competition: finishing laps with randomized obstacles and pulling off a flawless parallel parking manoeuvre. Teams must incorporate cutting-edge robotics ideas with an emphasis on innovation and dependability, such as computer vision, sensor fusion, and kinematics.

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



The propulsion system relies on a DC motor paired with an encoder. The motor provides the mechanical force required to move the vehicle, while the encoder outputs two pulse signals (c!
hannels A and B) that indicate the rotation direction and speed by measuring the number of pulses per revolution.
We used these motors for their balance between power, efficiency, and cost, as well as their easy integration with common motor drivers like the L298N.


**Servo Motor MG995:**
| Specifications: |
| ------------- |
| Voltage: 4.8-6V |
| Torque: 9.4/11 kg/cm |
| Rotation: 180° ± 10°  |
![617BvnN0VJL _SL1500_](https://github.com/user-attachments/assets/b501ca45-12d2-46de-9c1c-de0f9dc0fb90)


A servomotor is integrated into the system to control steering. The servo receives PWM signals from a dedicated Arduino Mega pin, allowing precise angular positioning between 0° and 180°. The servo’s position is controlled programmatically to perform smooth and accurate movements.
We chose this servo for its precise position control, ease of use with Arduino libraries, and widespread availability, offering better accuracy and reliability than cheaper or less-documented servos.

**Motor driver shield (L293):**
| Specifications: |
| ------------- |
| Power supply voltage: 4.5 to 25V  |
| Output current: 1.2A (peak) |
| Can handle: 4 DC motors or 2 stepper motors and 2 servo motors |
![mtr cntrlr](https://github.com/user-attachments/assets/fd5479d2-4d84-43ed-bd1c-b6e060d37bcf)

It is a driver board based on L293 IC, which can drive 4 DC motors and 2 stepper or Servo motors at the same time. Each channel of this module has the maximum current of 1.2A and doesn't work if the voltage is more than 25v or less than 4.5v.
The motor driver shield is designed to handle the power requirements of the DC motor and provide an easy interface for Arduino Mega. It accepts PWM signals for speed control and digital signals for motor direction, enabling forward and reverse movement. The shield includes built-in protection features such as current limiting and thermal shutdown to safeguard components during operation.
The L298N was selected because it can control two motors simultaneously, is robust, and works seamlessly with Arduino, unlike more complex or less compatible drivers.

### Power and Sense Management

**Arduino Mega 2560:**

|Specifications:|
| ------------ |
|Microcontroller: ATmega2560 |
|Flash memory: 256 kB |
|SRAM: 8 kB | 
|Frequency: 16 MHz |
|Pins: 56 |
|Input voltage: 5V |
![mega 2](https://github.com/user-attachments/assets/88007b6f-bfa6-4737-894a-474db0262f06)


Arduino Mega 2560: Is a microcontroller board based on the ATmega2560. It has 54 digital input/output pins and 16 analog inputs, a 16 MHz crystal oscillator, a USB connection, a power jack, an ICSP header, and a reset button. The arduino is the board that contains the code that allow us to accomplish the challenge, using the sensors data to make the necessary movements
We selected the Arduino MEGA for its user-friendly interface, extensive community support, and broad compatibility with various sensors and actuators, making development easier than with less-documented boards.

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

The camera is capable of detecting seven colors simultaneously and It is equipped with an internal processor, which lets us explore just the necessary information for the Arduino to evade in the necessary way, depending on the obstacle colour.
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

The code uses a state machine (`CarState`) for navigation, decision-making at intersections, straight driving, advanced filtering of sensor data, and dynamic corrections to maintain centering in a corridor. It is likely to use robust filtering for sensor readings and may feature improvements in navigation logic or integration with additional sensors.

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
- The state machine structure makes it easier to extend or modify navigation behaviors.
- Encoder and (if present) gyroscope integration enable precise movement and orientation correction.
- Update this documentation if the logic or hardware interfaces change.

---







