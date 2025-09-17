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
    - Servo Motor MG995
    - Module l298n motor driver
  - [Power and Sense Management](#power-and-sense-management)
    - ESP32 DevKit v1
    - ESP32 expansion board
    - Ultrasonic sensors (hc-sr04)
    - 18650 Battery
    - Pixy Cam v2
    - Voltage regulator
- [Software/Code Documentation – CodigoDeluxe4_0.ino](#softwarecode-documentation--codigodeluxe40ino)
  - [Open challenge](#open-challenge)
    - [High-level summary](#high-level-summary)
    - [Includes and global objects](#includes-and-global-objects)
    - [Pin and constant definitions (as declared)](#pin-and-constant-definitions-as-declared)
    - [Pixy2 helper](#pixy2-helper)
    - [Ultrasonic functions](#ultrasonic-functions)
      - [filtrarUltrasonicos(NewPing &sonar)](#filtrarultrasonicosnewping--sonar)
      - [readUltrasonicSensors()](#readultrasonicsensors)
    - [Servo helper functions](#servo-helper-functions)
    - [Motor control functions](#motor-control-functions)
      - [setMotorDirection(bool forward)](#setmotordirectionbool-forward)
      - [stopMotor()](#stopmotor)
      - [MotorSpeed(bool direccion, int velocidad)](#motorspeedbool-direccion-int-velocidad)
    - [Encoder-based movement functions](#encoder-based-movement-functions)
      - [EncoderForward(int encoderTicks)](#encoderforwardint-encoderticks)
      - [EncoderBackward(int encoderTicks)](#encoderbackwardint-encoderticks)
    - [Turn routines (composed actions)](#turn-routines-composed-actions)
    - [Small correction routines](#small-correction-routines)
    - [backwardFinal()](#backwardfinal)
    - [setup()](#setup)
    - [loop() and finite-state machine (FSM)](#loop-and-finite-state-machine-fsm)
      - [START](#start)
      - [SENSE](#sense)
      - [RIGHT](#right)
      - [LEFT](#left)
      - [STRAIGHT](#straight)
      - [STOPPED](#stopped)
      - [default](#default)
    - [Variables involved in control flow](#variables-involved-in-control-flow)
    - [How the pieces interact at runtime (short narrative)](#how-the-pieces-interact-at-runtime-short-narrative)
- [Cost Report](#cost-report)
  - [Components](#components)
  - [3D Printing Costs](#3d-printing-costs)
  - [Other Parts Tested](#other-parts-tested)
  - [Tools and Equipment](#tools-and-equipment)
  - [Summary of Costs](#summary-of-costs)
- [WRO SuperNova — Complete Assembly and Wiring](#wro-supernova--complete-assembly-and-wiring)
  1. [Quick overview](#1--quick-overview)
  2. [BOM](#2--bill-of-materials)
  3. [Tools](#3--tools)
  4. [PLA printing — settings & guidance (complete)](#4--pla-printing--complete-settings--guidance)
  5. [Hole, insert & fastener strategy for PLA](#5--hole-insert--fastener-strategy-for-pla-complete)
  6. [Part-specific print & post-process notes](#6--part-specific-print--postprocess-notes)
  7. [Mechanical assembly — step-by-step](#7--mechanical-assembly--stepbystep)
  8. [Electrical wiring — pin-by-pin](#8--electrical-wiring--pin-by-pin-match-to-code)
  9. [Power, decoupling and protection](#9--power)
  10. [Software, calibration & commissioning](#10--software-calibration--commissioning)


## Contents of the repository
· `t-photos`:Includes 2 photos of the team (one official and one fun with all members). 

`v-photos`: Contains 6 photos of the vehicle (from all angles, top and bottom).  

`video` : Video.md file with link to a driving demonstration video. 

`schemes`: Schematic diagrams (JPEG, PNG or PDF) of the electromechanical components, illustrating the connection of electronic elements and motors. 

`src`: Control software code for all components programmed for the competition.

`models`: Files for models used by 3D printers, laser cutters and CNC machines to produce vehicle elements.

`other`: Additional files to understand how to prepare the vehicle for competition (SBC/SBM connection documentation, file upload, hardware specifications, etc.), old code and the images on the repository.


the Project
===
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. the vehicle takes different paths depending on the color of the obstacle. We use an ESP32-based program (C++) that includes codes for the ultrasonic sensors (hc-sr04), which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the ESP32 circuit board, which then directs the vehicle’s movement system to turn det either clockwise or counterclockwise, as pre-established. Additionally, the vehicle is equipped with a camera module (Pixy v2) to detect colored obstacles and avoid them based on their color

the core of the system is based on the ESP32 v1 devkit microcontroller, which provides a versatile platform with sufficient input/output pins and processing capability to manage multiple sensors and actuators for basic robotics applications. the vehicle features a DC motor equipped with an encoder, allowing for precise feedback on wheel rotation to enable closed-loop speed and distance control. Four ultrasonic sensors are strategically mounted to the vehicle, giving comprehensive environmental awareness by measuring distances to obstacles in front and on both sides. A motor driver shield is used to manage power delivery and control signals to the motor, while a servomotor handles steering actuation. the chassis for this vehicle was custom-designed using 3D modeling software and subsequently fabricated with a 3D printer.



## the Challenge
Teams are challenged to create, assemble, and program a robotic car that can drive itself around a racetrack that is dynamically altered for every round. there are two primary objectives in the competition: finishing laps with randomized obstacles and pulling off a flawless parallel parking manoeuvre. Teams must incorporate cutting-edge robotics ideas with an emphasis on innovation and dependability, such as computer vision, sensor fusion, and kinematics.

This challenge emphasizes all aspects of the engineering process, including:

- Mobility Management: Developing efficient vehicle movement mechanisms.
- Obstacle Handling: Strategizing to detect and navigate traffic signs (red and green markers) within specified rules.
- Documentation: Showcasing engineering progress, design decisions, and open-source collaboration through a public GitHub repository.

## Photos of DELTA

| Front view | Back view | Left view | 
| ------------- |------------- | ------------- |
| ![DELTA Front](https://github.com/user-attachments/assets/6b75534f-0f5e-4092-8571-abf5b8a0d158)| ![DELTA Back](https://github.com/user-attachments/assets/52d4785b-2a04-40ab-bfa1-ace6d514cec9)| ![DELTA Left](https://github.com/user-attachments/assets/4ff15c7c-de28-4c58-83e0-6b2941a30886)|

| Right view | Top view | Bottom view |
| ------------- | ------------- |------------- |
|![DELTA Right](https://github.com/user-attachments/assets/4fad3d60-001c-4e51-b5a7-5031d2bf155f)| ![DELTA Top](https://github.com/user-attachments/assets/2741638f-efc4-4f0a-b089-daeeddca627b) |![DELTA Bottom](https://github.com/user-attachments/assets/dbf5b66c-b878-4635-9ba1-8910e8f668d6) |


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


A servomotor is integrated into the system to control steering. the servo receives PWM signals from a dedicated ESP32 pin, allowing precise angular positioning between 0° and 180°. the servo’s position is controlled programmatically to perform smooth and accurate movements.
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


It is a sensor that uses ultrasonic sounds to detect the bounce time of sound from one side to the other. Using the ESP32 devkit v1 we can determine the distance based on the time it takes for the wave to return, performing the function of determining when there is a wall nearby, and thus making the corresponding turn.
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

we are going to use 3 of these li-ion batterys in parallel to power the vehicle, with an 18650 battery holder. This battery configuration provides a reliable and efficient power source for the robot’s electronics and actuators, supporting extended operation during competition or testing.


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

the camera is capable of detecting seven colors simultaneously and It is equipped with an internal processor, which lets us explore just the necessary information for the ESP32 to evade in the necessary way, depending on the obstacle colour.
 We selected this camera for its high-resolution image capture, easy interfacing with microcontrollers, and strong community support, providing superior performance and documentation compared to less common camera modules.

**Voltaje Regulator (Lm2596) :**
| Specifications: |
| ------------- |
| Input Voltage: 4V to 35V |
| Output Voltage:  1.23V to 37V |
| Energy eficiency : 80% |
![voltage regulator](https://github.com/user-attachments/assets/7e0a2466-8ae1-4540-9adb-790cd3f9cec8)

The module LM2596 is a regulator step down that can reduce de voltage of input to a lower voltage in output 

# Software/Code Documentation – `CodigoDeluxe4_0.ino`

This document describes the structure, logic, and key functions of the file: `src/CodigoDeluxe4_0.ino` in the WRO-SuperNova project.

---
# Open challenge 

## High-level summary
Firmware for an ESP32-based robot that:
- Reads four ultrasonic sensors (front, right, left, rear) using the NewPing library.
- Uses an ESP32 encoder for distance/tick measurements.
- Controls a DC motor through two direction pins and an enable pin (ENB).
- Uses a single servo for steering with three preset positions (left, center, right).
- Implements a finite-state machine to drive, sense obstacles, decide turns, execute turns using encoder distances, and count laps.

---

## Includes and global objects
- Libraries:
  - `Arduino.h`, `WiFi.h`, `Pixy2.h`, `NewPing.h`, `ESP32Encoder.h`, `ESP32Servo.h`.
- Global peripheral objects:
  - `Pixy2 pixy;`
  - `NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);` — right ultrasonic
  - `NewPing sonarLeft(...)` — left ultrasonic
  - `NewPing sonarFront(...)` — front ultrasonic
  - `NewPing sonarRear(...)` — rear ultrasonic
  - `Servo servo;` — steering servo
  - `ESP32Encoder encoder(true);` — encoder instance (quadrature/direction enabled)

---

## Pin and constant definitions (as declared)
- Motor:
  - DIR_PIN_A = 16
  - DIR_PIN_B = 17
  - pinENB = 5
- Ultrasonics:
  - Right: TRIGGER 33, ECHO 25
  - Left:  TRIGGER 27, ECHO 26
  - Front: TRIGGER 12, ECHO 14
  - Rear:  TRIGGER 22, ECHO 23
- Servo pin: 2
- Distances, thresholds and tuning variables:
  - MAX_DISTANCE = 300
  - servoL = 115, servoC = 80, servoR = 50
  - frontObject = 90
  - cantMuestras = 5 (samples per ultrasonic sensor read)
  - DELAY_ULTRASONICOS = 10 (ms)
  - wallfence = 20
  - laps = 0
  - sense = 0 (direction chosen during sensing: 1 or 2)
  - maximumCorrections = 0

---

## Pixy2 helper
- `getSignatureColorName(int signature)`
  - Returns a human-readable name for a Pixy signature id.
  - Cases: `1 -> "Verde"`, `2 -> "Rojo"`, default -> `"Desconocido"`.
- (There are commented-out helper functions that format Pixy block data as HTML; they call `pixy.ccc.getBlocks()` and iterate `pixy.ccc.blocks` to extract signature, x/y/width/height.)

---

## Ultrasonic functions

### filtrarUltrasonicos(NewPing &sonar)
- Purpose: take multiple readings from a single NewPing sonar and return a filtered average distance in cm.
- Behavior:
  - Performs `cantMuestras` readings using `sonar.ping_cm()`.
  - Treats `0` returned by `ping_cm()` as `MAX_DISTANCE`.
  - Keeps running `sum`, tracks `minVal` and `maxVal`, and counts samples (`validCount`).
  - After sampling, subtracts `minVal` and `maxVal` from `sum` (trim one min and one max) and divides by `validCount - 2` (if `validCount >= 3`) or `validCount` otherwise.
  - Returns the computed average as `unsigned int`.
- Notes on outputs: returns a value in cm bounded by `MAX_DISTANCE`.

### readUltrasonicSensors()
- Purpose: update global distances for all four ultrasonic sensors.
- Behavior: calls `filtrarUltrasonicos()` for each sonar and stores results in:
  - `distanceFront`, `distanceRight`, `distanceLeft`, `distanceRear`.
- After reading, any zero distances are set to `MAX_DISTANCE` (safeguard).

---

## Servo helper functions
- `servoRigth()` — write servo angle `servoR` (steer right) and short `delay(15)`.
- `servoLeft()` — write servo angle `servoL` (steer left) and short `delay(15)`.
- `servoCenter()` — write servo angle `servoC` (center steering) and short `delay(15)`.

These functions position the steering servo to one of three predefined angles.

---

## Motor control functions

### setMotorDirection(bool forward)
- Sets motor direction pins for forward or backward motion:
  - If `forward == true`: DIR_PIN_A = LOW, DIR_PIN_B = HIGH.
  - Else: DIR_PIN_A = HIGH, DIR_PIN_B = LOW.

### stopMotor()
- Stops the motor by setting DIR_PIN_A = LOW, DIR_PIN_B = LOW and pinENB = LOW.

### MotorSpeed(bool direccion, int velocidad)
- Calls `setMotorDirection(direccion)` to choose direction.
- Writes `velocidad` to `pinENB` using `digitalWrite(pinENB, velocidad)`.
- Intended to control motor speed and direction together.

---

## Encoder-based movement functions

### EncoderForward(int encoderTicks)
- Purpose: move forward until the encoder count reaches `encoderTicks`.
- Behavior:
  - Calls `encoder.clearCount()` to reset the encoder count.
  - While `encoder.getCount() < encoderTicks`:
    - Calls `MotorSpeed(true, 150)` to move forward.
    - Delays 10 ms inside loop.
  - Calls `stopMotor()` and `encoder.clearCount()` at the end.

### EncoderBackward(int encoderTicks)
- Purpose: move backward until the encoder count goes beyond `encoderTicks`.
- Behavior:
  - Calls `encoder.clearCount()`.
  - While `encoder.getCount() > encoderTicks`:
    - Calls `MotorSpeed(false, 150)` to move backward.
    - Delays 10 ms inside loop.
  - Calls `stopMotor()` and `encoder.clearCount()` at the end.

Notes:
- These functions use the encoder reading to control distances moved by repeated on/off motor commands until the encoder target is reached.

---

## Turn routines (composed actions)
Each routine uses the encoder move functions and servo to perform a turn maneuver.

- `Lbig()`:
  - Calls `EncoderBackward(-700)` (back up),
  - Calls `servoLeft()` (steer left),
  - Calls `EncoderForward(1300)` (move forward the turn distance),
  - Calls `servoCenter()` (re-center).

- `Lsmall()`:
  - Calls `EncoderBackward(-650)`,
  - `servoLeft()`,
  - `EncoderForward(1300)`,
  - `servoCenter()`.

- `Rbig()`:
  - Calls `EncoderBackward(-700)`,
  - `servoRigth()` (steer right),
  - `EncoderForward(1300)`,
  - `servoCenter()`.

- `Rsmall()`:
  - Calls `EncoderBackward(-650)`,
  - `servoRigth()`,
  - `EncoderForward(1300)`,
  - `servoCenter()`.

These routines perform: back up slightly, set steering to turn direction, move forward a fixed encoder distance to complete the turn, then re-center the servo.

---

## Small correction routines
- `correctionL()`:
  - Set servo to left,
  - Call `MotorSpeed(true, 120)` to move forward briefly,
  - `delay(200)`,
  - `servoCenter()` and `delay(15)`.

- `correctionR()`:
  - Set servo to right,
  - `MotorSpeed(true, 120)`,
  - `delay(200)`,
  - `servoCenter()` and `delay(15)`.

These move the robot slightly forward while steering to perform small lateral corrections.

---

## backwardFinal()
- Calls `Lbig()` if `sense == 1`.
- Calls `Rbig()` if `sense == 2`.
- Used as a final backup-and-turn routine when stopping.

---

## setup()
- Initializes Serial at 115200.
- Configures motor pins DIR_PIN_A, DIR_PIN_B, pinENB as OUTPUT.
- Attaches servo on `servoPin` with pulse width range 500–2500 µs.
- Initializes Pixy2: `pixy.init()`.
- Configures encoder:
  - Enables internal pull-ups: `ESP32Encoder::useInternalWeakPullResistors = puType::up`.
  - Attaches encoder pins with `encoder.attachSingleEdge(4, 5)`.
  - Clears encoder count `encoder.clearCount()`.
- Initializes `lastStateChangeTime = millis()` (used for potential state timing).

---

## loop() and finite-state machine (FSM)
`loop()` repeatedly:
- Calls `readUltrasonicSensors()` to update distanceFront, distanceLeft, distanceRight, distanceRear.
- Updates `encoderTicks = encoder.getCount()`.
- Uses `switch(currentState)` to run the FSM. States:

### START
- Centers the servo.
- Reads ultrasonic sensors.
- Sets `sense = 0`.
- Calls `MotorSpeed(true, 200)` to move forward.
- If `distanceFront < frontObject` and `distanceFront != MAX_DISTANCE` and `sense == 0`:
  - Calls `stopMotor()`.
  - Sets `currentState = SENSE`.

### SENSE
- Decides turning direction by comparing lateral distances:
  - If `distanceLeft < distanceRight`:
    - Sets `sense = 2` (right) and `currentState = RIGHT`.
  - If `distanceRight < distanceLeft`:
    - Sets `sense = 1` (left) and `currentState = LEFT`.

### RIGHT
- If `laps < 11`:
  - If `distanceLeft < 70` and `distanceLeft > 0`:
    - `laps += 1; Rbig(); readUltrasonicSensors(); maximumCorrections = 0; currentState = STRAIGHT;`
  - Else if `70 < distanceLeft < 100`:
    - `laps += 1; Rsmall(); readUltrasonicSensors(); maximumCorrections = 0; currentState = STRAIGHT;`
- Else if `laps >= 11`:
  - `currentState = STOPPED`.

### LEFT
- Symmetric behavior to RIGHT but uses `distanceRight` for decisions:
  - If `distanceRight < 70` and `distanceRight > 0` => `Lbig()` etc.
  - If `70 < distanceRight < 100` => `Lsmall()` etc.
  - If `laps >= 11` => `currentState = STOPPED`.

### STRAIGHT
- Calls `MotorSpeed(true, 70)` to drive forward at a slower speed.
- Calls `readUltrasonicSensors()` then:
  - If `distanceFront < frontObject` and `sense == 1` and `distanceFront > 50`:
    - `currentState = LEFT`.
  - If `distanceFront < frontObject` and `sense == 2` and `distanceFront > 50`:
    - `currentState = RIGHT`.
  - If `distanceFront < 10`:
    - Decrements `laps`, moves backward for a short fixed time (`MotorSpeed(false, 200)` + `delay(1500)`), then stops motor.
- Lateral corrections within STRAIGHT:
  - If `distanceLeft < wallfence` and `maximumCorrections < 2`: increment `maximumCorrections` and call `correctionR()`.
  - If `distanceRight < wallfence` and `maximumCorrections < 2`: increment `maximumCorrections` and call `correctionL()`.

### STOPPED
- Calls `backwardFinal()` (final backup & turn depending on `sense`).
- Calls `stopMotor()`.
- `delay(50000)` to remain stopped for a period.

### default
- Sets `currentState = STRAIGHT`.

---

## Variables involved in control flow
- `currentState` (enum CarState): `START`, `STOPPED`, `SENSE`, `RIGHT`, `LEFT`, `STRAIGHT`.
- `sense`: direction decision (1 = left, 2 = right, 0 = none).
- `laps`: increments each time a corner/turn is executed; used to limit runs.
- `maximumCorrections`: counter for lateral corrections applied while moving straight.

---

## How the pieces interact at runtime (short narrative)
- The robot begins in `START`, moves forward, and monitors the front ultrasonic sensor.
- When an obstacle is detected within `frontObject`, it transitions to `SENSE` and compares left/right distances to choose a turn direction (`sense`).
- It executes the chosen turn (big or small) using encoder-based forward/backward moves combined with steering via servo.
- After turning it enters `STRAIGHT` to follow the corridor, apply lateral corrections, and watch for the next obstacle to repeat the cycle.
- `laps` is incremented on each corner; when a target number of laps is reached the FSM transitions to `STOPPED` and performs a final backup-and-turn before stopping.

---

# Close challenge 

## 1 — High-level summary
This sketch runs on an ESP32 and coordinates a Pixy2 camera, four ultrasonic sensors (NewPing), a steering servo, an encoder, and a DC motor controlled by an H-bridge. The program implements a finite-state machine to search for colored blocks (green/red) using Pixy2 and execute preprogrammed maneuvers when colors are detected.

Main flow: START → WALL → SEARCH → (GREEN or RED) → back to SEARCH or WALL. Flags `G` and `R` remember whether a maneuver for green or red has already been executed.

---

## 2 — Pin mapping
- Motor:
  - DIR_PIN_A = GPIO16
  - DIR_PIN_B = GPIO17
  - pinENB = GPIO32 (H-bridge enable / speed)
- Servo:
  - servoPin = GPIO2
- Ultrasonic sensors (NewPing):
  - Right TRIG / ECHO = GPIO33 / GPIO25
  - Left TRIG / ECHO = GPIO27 / GPIO26
  - Front TRIG / ECHO = GPIO13 / GPIO14
  - Rear TRIG / ECHO = GPIO21 / GPIO34
- Encoder:
  - encoder.attachSingleEdge(4, 22) → channel A GPIO4, channel B GPIO22
- Pixy2:
  - object `pixy` used; blocks retrieved with `pixy.ccc.getBlocks()` and read via `pixy.ccc.blocks[]`

---

## 3 — Key variables and constants
- Servo angles: `servoL = 115`, `servoC = 80`, `servoR = 50`
- Ultrasonic parameters:
  - `MAX_DISTANCE = 300` cm
  - `cantMuestras = 5` samples per sensor
  - `DELAY_ULTRASONICOS = 10` ms between pings
  - `wallfence = 30` cm (distance considered "close" to a wall)
  - `frontObject = 90` (front threshold referenced in other sketches)
- Motion/encoder:
  - `ppr = 1400` pulses per revolution (not directly used here)
  - `laps`, `diameter` (context variables, not central here)
- Color flags:
  - `G` and `R` are 0/1 flags tracking green/red maneuvers
- `sense` determines preferred turn direction during the WALL routine
- `pixy.ccc.numBlocks` contains the number of color blocks detected by Pixy

---

## 4 — Main objects
- `Pixy2 pixy` — camera block detection (color signatures)
- `NewPing sonarRight/Left/Front/Rear` — ultrasonic sensors
- `ESP32Encoder encoder(true)` — quadrature encoder counter
- `Servo servo` — steering servo

---

## 5 — Important functions (what they do)

### filtrarUltrasonicos(NewPing &sonar)
- Takes `cantMuestras` readings with `sonar.ping_cm()`.
- Replaces 0 with `MAX_DISTANCE`.
- Sums values, discards min and max (if enough samples), and returns the averaged result.
- Purpose: reduce spurious readings / outliers.

### readUltrasonicSensors()
- Calls `filtrarUltrasonicos()` for each sensor and sets `distanceFront/Right/Left/Rear`.
- Ensures that a returned 0 is converted to `MAX_DISTANCE`.

### servoRight(), servoLeft(), servoCenter()
- Move the servo to preconfigured angles and wait 15 ms.

### setMotorDirection(bool forward), stopMotor(), MotorSpeed(bool direccion, int velocidad)
- `setMotorDirection` sets DIR pins for forward/backward.
- `stopMotor` sets both DIR pins LOW and `pinENB` LOW.
- `MotorSpeed` sets direction and calls `analogWrite(pinENB, velocidad)` to control speed.
  - Note: `analogWrite` is not the standard Arduino API on the ESP32 core — LEDC (ledcSetup/ledcAttachPin/ledcWrite) is recommended.

### EncoderForward(int encoderTicks), EncoderBackward(int encoderTicks)
- Clear encoder count, then loop while encoder count hasn't reached target, commanding MotorSpeed inside the loop and delaying.
- After reaching target, call stopMotor() and clear encoder count.
- These functions are blocking and currently have no timeout safeguards.

---

## 6 — State machine (detailed)

### START
- Centers servo, sets `sense = 1`, and transitions to `WALL`.
- Acts as a simple initialization step and chooses a default turning sense (right).

### WALL
- Moves forward with `MotorSpeed(true, 200)` for 2 seconds, then stops.
- Based on `sense`:
  - If `sense == 1`: steer right, reverse for 2 seconds, center servo, `EncoderForward(300)`, go to `SEARCH`.
  - If `sense == 2`: same but mirrored (left).
- Purpose: reposition the robot away from a wall and get into a search posture.
- Note: there is commented code that would choose `sense` based on left vs right distances; currently `sense` is forced in START.

### SEARCH
- Iterates over `pixy.ccc.numBlocks` and:
  - If `m_signature == 1` → set state to `GREEN`.
  - If `m_signature == 2` → set state to `RED`.
  - Else (inside the for) it calls `MotorSpeed(true, 200)` and delays 200 ms.
- Issue: If `pixy.ccc.numBlocks == 0`, the for-loop does not run and the robot will not advance. The intended behavior probably is: if no blocks detected, advance; otherwise process blocks. The current placement of MotorSpeed inside the `else` of the for is problematic.

### GREEN
- Behavior depends on flags `R` and `G`:
  - If `R == 1`: sequence Left → forward 400, Right → forward 900, Center → forward 100; set `G = 0`, return to `SEARCH`.
  - If `(G == 0) && (R == 0)`: Left → forward 700, Right → forward 700, Center → forward 100; set `G = 1`, return to `SEARCH`.
  - Else if `G == 1`: move slowly `MotorSpeed(true, 30)`; if `distanceFront < 20` → set `G = 0`, go to `WALL`.
- Purpose: first detection triggers a larger exploratory maneuver (marking `G = 1`), subsequent detections switch to slow advance until a frontal obstacle is detected.

### RED
- Symmetric to GREEN but using the `R` flag:
  - If `G == 1`: perform a maneuver, clear `G = 0`, return to `SEARCH`.
  - If `(R == 0) && (G == 0)`: perform a maneuver, set `R = 1`, return to `SEARCH`.
  - If `R == 1`: move slowly; if `distanceFront < 20` then `R = 0`, transition to `WALL`.

### STOPPED
- Present in the enum but not used.

---



---



# Cost Report
## Components
Component	Quantity	Cost per Unit ($)	Total ($)
| Componets | Component	Quantity  | Cost per Unit ($) | Total ($) |
| ------------- | ------------- |------------- |------------- |
| Motor DC GS20179-09 | 1 | 20.79 | 20.79 |
| Servo Motor MG995 | 1 | 3.71 | 3.71 |
| Module l298n motor driver | 1 | 4.99 | 4.99 |
| ESP32 DevKit v1 | 1 | 15.99 | 15.99 |
| ESP32 expansion board | 9.99 | 9.99 |
| Ultrasonic sensors (hc-sr04) | 4 | 4.99 | 15.96 |
| EBL 18650 Battery 3.7v 3300mha | 3 | 5.49 | 16.47 |
| PixyCam v2 | 1 | 69.90 | 69.90 | 
| Voltaje Regulator (Lm2596) | 1 | 4.52 | 4.52 | 
| Wheels of 2.559 | 2 | 4.99 | 9.98 |   
| 18650 3 battery holder | 1 | 1.99 | 1.99 |
| DuPont cable jumper | 1 kit | 7.99 | 7.99 |

**Total for Components: 182.28$**

## 3D Printing Costs
Filament Used:
Prototypes: 1.5 kg of PLA filament
Final Parts: 250g of PLA filament
PLA Filament Cost: 25.00 $  per 1kg
| Filament Use | Weight (g)	| Cost ($) |
| ------------- | ------------ | ----------- |
| Prototypes | 1,500 | 45.00 | 
| Final Parts	| 250 | 7.45 |

**Total for 3D Printing: 52.45$**

## Other Parts Tested
**Approximate Cost for Additional Parts Tested: 472.00$**
## Tools and Equipment
| Tool | Cost ($) |
| ---- | ---------|
| Bambu Lab A1 mini 3D Printer | 299.00 |
| Soldering iron | 11.99 |
| Multimeter | 18.00 |
| Miscellaneous Tools | 39.00 |

**Total for Tools and Equipment: 367.00$**

## Summary of Costs
| Category | Total ($) | 
| --------- | --------- | 
| Components | 174.29 |
| 3D Printing | 52.45 |
| Other Parts Tested | 472.00 |
| Tools and Equipment | 367.00 |
| Shipping Approximation | 10.00 |

**Grand Total: 1.082,99$**

*Note: Costs are approximate and based on current exchange rates and market prices.*

# WRO SuperNova — Complete Assembly and Wiring

This is the single, canonical assembly guide for DELTA. It covers everything from printing settings to mechanical assembly, wiring to the corrected firmware (ENB → GPIO13 PWM), safety, commissioning and troubleshooting — all in one place so you can follow it end-to-end.

Use the repo images for reference during assembly:
- other/Images of the readme/ModuloESP32-DEVKITV1-30pines-min_1_2048x2048.jpg
- other/Images of the readme/Base expansora para ESP32.jpg
- other/Images of the readme/Ultrasonic_Sensor.jpg
- other/Images of the readme/pixy-v21-camera-sensor.jpg
- other/Images of the readme/voltage regulator.jpg
- v-photos/ The DELTA photos for mechanical orientation
- schemes/ use the schemes for cable orientation 

Repository firmware this guide matches:
- CodigoDeluxe4_1.ino

---

Table of contents
1. Quick overview
2. BOM 
3. Tools
4. PLA printing — settings & guidance (complete)
5. Hole, insert & fastener strategy for PLA
6. Part-specific print & post‑process notes
7. Mechanical assembly — step‑by‑step (with wiring cues)
8. Electrical wiring — pin-by-pin
9. Power, decoupling and protection
10. Software, calibration & commissioning

---

1 — Quick overview
This guide assumes you will print almost all robot mechanical parts in PLA. PLA works well: good dimensional accuracy, stiff parts, and easy printing. This guide addresses PLA limitations (heat sensitivity, reduced screw-hold longevity) and gives methods to make a reliable robot while staying .

Firmware/pin reference:
- Motor DIR A → GPIO16
- Motor DIR B → GPIO17
- Motor EN (PWM) → GPIO13 
- Servo signal → GPIO2 (MG995)
- Encoder A/B → GPIO4 / GPIO5
- Ultrasonics:
  - Front TRIG/ECHO → GPIO12 / GPIO14
  - Right TRIG/ECHO → GPIO33 / GPIO25
  - Left TRIG/ECHO → GPIO27 / GPIO26
  - Rear TRIG/ECHO → GPIO22 / GPIO23
- Pixy2 → SPI recommended (3.3V Vcc); connect per Pixy docs

---

2 — Bill of Materials 
Mechanical
- Robot CarDC V4 chasis .stl — bottom chassis
- Chasis superior V2.stl — top cover
- Direccionales Robot Chasis V4.stl — steering mounts / knuckles
- MG995.stl (horn/bracket) or use the servo-supplied horn
- Ruedas V2.stl

Electronics
- ESP32 DevKit (30-pin)
- Motor driver L298n
- expansion board v1 for ESP32
- DC motor with encoder and wheels
- MG995 or similar servo
- Pixy2 camera module
- 4 × HC-SR04 ultrasonic sensors 
- Power: 18650 3000mh x3
- Wiring: DuPont jumper wires
  
Consumables & fasteners
- M3 machine screws (assorted lengths) + M3 nuts
- Zip‑ties, foam tape, cable clips

---

3 — Tools
- 3D printer (FDM) configured for PLA
- Small screwdriver set (PH0/PH1), hex keys
- Soldering iron + shrink tubing
- Multimeter
- Wire stripper/crimper, pliers
- Drill or reamer (3.0–3.2 mm) for hole finishing
- Heat gun/hot air or soldering iron for low-temp insert installation (use with care)
- Calipers for dimensional checks

---

4 — PLA printing — complete settings & guidance

Printer nozzle & bed
- Nozzle: 0.4 mm
- Bed: PEI, glass or taped bed works fine
- Nozzle temp: 200–210 °C (start 205 °C; tune by filament)
- Bed temp: 55–60 °C
- Fan: 100% after first 2–3 layers

Layer & perimeters
- Layer height: 0.16–0.20 mm (0.12 mm for small high-detail pivots)
- Perimeters: 3–4 (4 recommended for chassis and load zones)
- Top/bottom layers: 6

Infill
- Chassis & load-bearing parts: 35–40% gyroid or grid
- Brackets/covers: 15–25%
- Pivot/knuckle areas: 30–40%

Speeds & retraction
- Print speed: 40–50 mm/s
- Perimeter speed: 30–40 mm/s for stronger walls
- Retraction: Bowden 4–6 mm / Direct 0.8–1.2 mm; retraction speed 25–40 mm/s

Adhesion & supports
- Brim: 3–8 mm for large chassis parts
- Supports: tree supports for overhangs; minimize contact with cosmetic surfaces
- First layer: 20–25 mm/s, slightly higher extrusion multiplier if needed

Slicer tuning tips
- Use 4 perimeters for screw bosses; enable “pause after layer” if you want to embed nuts mid-print.
- Turn on “Z hop” only if needed; it can reduce small scuffs.
- Enable “coast at end” and “retraction in wipe” if stringing appears.

Cooling & warping
- PLA benefits from active cooling; ensure 100% fan once layers are stable.
- For long prints, avoid drafts or sudden ambient temp changes.

---

5 — Hole, insert & fastener strategy for PLA (complete)

PLA characteristics
- Stiff and dimensionally accurate, but more brittle and less heat-resistant than PETG.
- Repeated screwing into raw PLA will wear threads.

Recommended fastening approaches

Hole prep
- Print holes slightly undersized.
- Ream or drill to 3.0–3.2 mm for M3 clearance if using machine screws with nuts.
- For screws threading into PLA, pilot holes 2.6–2.8 mm are good for M3 self-tapping screws.

---

6 — Part-specific print & post‑process notes

A) Main chassis — Robot CarDC V4 chasis .stl
- Orientation: flat on the bed — largest surface down.
- Settings: 0.16–0.20 mm layer, 4 perimeters, 40% infill, brim 5–8 mm.
- Post: ream screw holes, remove any internal supports, chamfer screw entry slightly for easier insertion.

B) Top layer — Chasis superior V2.stl
- Orientation: as modeled for correct standoffs up.
- Settings: 0.16–0.20 mm layer, 3 perimeters, 20% infill.
- Post: test-fit standoffs and ESP32/devkit.

C) Direccionales (steering) — Direccionales Robot Chasis V4.stl
- Orientation: pivot holes vertical (hole axis along Z) for best circularity.
- Settings: 0.12–0.16 mm layer for pivot detail, 4 perimeters around pivots, 30% infill.
- Post: sand pivot bores lightly; test-fit bearings/bushings.

D) Servo horn & bracket — MG995.stl
- Orientation: horn flat on bed.
- Settings: 0.12–0.16 mm layer, 4 perimeters, 30% infill.
- Post: test fit on servo spline; if tight, sand minimal.

---

7 — Mechanical assembly — step‑by‑step

Preparation (before fastening)
- Clean all prints, deburr, ream holes as needed, and gather hardware and electronics.
- Label wires (TRIG/ECHO, motor, servo) with tape for easy routing.

Assembly Steps
1) Install the motor into bottom chassis
   - Mount motor in motor mount provided; use M3 screws with nuts. Keep motor wire exit toward the side for short routing, add the smaller gear into the motor.

2) Mount wheel hubs and steering module
   - Place the inside bearing, then the bar and the bigger gear and the outside bearing, mount the wheels and the use the bottom photo of delta fro better guidance for the steering module

3) Mount motor driver 
   - Place motor driver close to motor to minimize VMOT wiring.

4) Mount ESP32 DevKit
   - Place ESP32 on the expansion board/expansion hat in the designated place in the top layer.
   - Secure with M2.5/M3 screws and nuts; use a captive nut if possible.

5) Mount servo (MG995)
   - Place servo in its bracket, secure with M3 screws.
   - Attach horn matching steering arm and center servo before final tightening.

6) Install ultrasonic sensors
   - Insert HC‑SR04 modules into their cutouts with front facing outward.
   - Secure with screws or double-sided tape; ensure sensor face is unobstructed.

7) Mount Pixy2 camera
   - Attach Pixy2 on front bracket; route its cable toward the ESP32 hat area.
   - the Pixy needs a designated regulator for the 5V

8) Connect encoder wires
   - Route encoder A/B to GPIO4/GPIO5; keep wires short and twisted.

9) Connect servo and sensors
   - Route servo signal to GPIO2; servo Vcc to dedicated 5–6V supply (do not power servo from weak 5V regulators).

10) Place top layer & fasten
   - attach the top layer, secure with screws (.

11) Cable routing & strain relief
   - Use zip ties; ensure no wires touch moving parts.
   - Keep high-current motor wires separate from sensor wires.

---

8 — Electrical wiring — pin-by-pin (match to code)

Pin mapping:
- Motor DIR A (IN1) ← GPIO16
- Motor DIR B (IN2) ← GPIO17
- Motor EN (PWM ENB) ← GPIO13 (LEDC ch0)
- Servo signal ← GPIO2 (MG995)
- Encoder A ← GPIO4
- Encoder B ← GPIO5
- HC-SR04 Front: TRIG → GPIO12, ECHO → GPIO14 (ECHO level-shifted)
- HC-SR04 Right: TRIG → GPIO33, ECHO → GPIO25 (ECHO level-shifted)
- HC-SR04 Left: TRIG → GPIO27, ECHO → GPIO26 (ECHO level-shifted)
- HC-SR04 Rear: TRIG → GPIO22, ECHO → GPIO23 (ECHO level-shifted)
- Pixy2: SPI recommended (MOSI/MISO/SCK/CS) — Pixy Vcc → 3.3V, GND → common GND

Wiring rules
- All grounds must be common: ESP32 GND, motor driver GND, servo GND, sensor GND.
- Use JST connectors for sensors & servo where practical to make removal easy.

Motor driver wiring
- VMOT (+) ← motor battery positive 
- VMOT GND ← motor battery negative
- IN1 ← GPIO16; IN2 ← GPIO17; EN ← GPIO13 (PWM)
- OUTA/OUTB ← motor terminals

---

9 — Power

Power rails
- 3 3.7v 18650 batteries in parallel going to each regulator for its designated components 

---

10 — Software, calibration & commissioning

Firmware
- Use the provided `CodigoDeluxe4_1.ino`.
- Ensure you uploaded the corrected sketch (ENB on GPIO13).

Note: To upload code from the Arduino IDE to an ESP32, first ensure the ESP32 board package is installed via the Boards Manager within the Arduino IDE.
 Select the correct ESP32 board model from the Tools > Board menu and choose the appropriate COM port under Tools > Port.
 For boards that do not enter flashing mode automatically, press and hold the BOOT button on the ESP32 while clicking the Upload button in the Arduino IDE; release the BOOT button once the upload process starts.
 If the upload fails with a timeout error, try pressing the on-board RST or EN button when the IDE shows "Connecting..." in the output console.
 After a successful upload, press the EN button to restart the ESP32 and run the new code.





