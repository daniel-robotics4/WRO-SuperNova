The Projet
===
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. The vehicle takes different paths depending on the color of the obstacle. We use an Arduino-based program (C++) that includes code for ultrasonic sensors, which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the Arduino circuit board, which then directs the vehicle’s movement system to turn either clockwise or counterclockwise, as pre-established. Additionally, the vehicle is equipped with a camera module (Pixy v2) to detect colored obstacles and avoid them based on their color

The core of the system is based on the Arduino Mega 2560 microcontroller, which offers ample input/output pins and processing power to handle multiple sensors and actuators simultaneously. The vehicle integrates a DC motor equipped with an encoder to provide precise feedback on wheel rotation, enabling closed-loop speed and distance control. Three ultrasonic sensors are strategically mounted to provide comprehensive environmental awareness by measuring distances to obstacles in front and on both sides of the vehicle. A motor driver shield manages power delivery and control signals to the motor, while a servomotor is used to actuate steering,the chasis made for this vehicle was made from scratch in a 3d tool an then printed

## The Challenge
Teams are challenged to create, assemble, and program a robotic car that can drive itself around a racetrack that is dynamically altered for every round. There are two primary objectives in the competition: finishing laps with randomized obstacles and pulling off a flawless parallel parking manoeuvre. Teams must incorporate cutting-edge robotics ideas with an emphasis on innovation and dependability, such as computer vision, sensor fusion, and kinematics.

This challenge emphasizes all aspects of the engineering process, including:

- Mobility Management: Developing efficient vehicle movement mechanisms.
- Obstacle Handling: Strategizing to detect and navigate traffic signs (red and green markers) within specified rules.
- Documentation: Showcasing engineering progress, design decisions, and open-source collaboration through a public GitHub repository.




## Management

### Mobility Mangement

**Motor DC 12v with encoder:**
| Specifications: |
| ------------- |
|  |
|  |
|  |
|  |
|  |
|  |


The propulsion system relies on a DC motor paired with an encoder. The motor provides the mechanical force required to move the vehicle, while the encoder outputs two pulse signals (channels A and B) that indicate the rotation direction and speed by measuring the number of pulses per revolution.

**Servo Motor 9g:**
| Specifications: |
| ------------- |
| voltage: 4.8-6V |
| torque: 1.6-1.8 kg/cm |
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

is a driver board based on L293 IC, which can drive 4 DC motors and 2 stepper or Servo motors at the same time. Each channel of this module has the maximum current of 1.2A and doesn't work if the voltage is more than 25v or less than 4.5v.
The motor driver shield is designed to handle the power requirements of the DC motor and provide an easy interface for Arduino Mega. It accepts PWM signals for speed control and digital signals for motor direction, enabling forward and reverse movement. The shield includes built-in protection features such as current limiting and thermal shutdown to safeguard components during operation.

### Power and Sense Management
**Arduino mega 2560:**
| Specifications: |
| ------------- |
| Microcontroller: ATmega2560 | 
| Flash memory: 256 KB |
| SRAM: 8 KB |
| Frequency: 16 MHz |
| Pins: 56 |
| Input voltage: 5V  |
 ![mega 2](https://github.com/user-attachments/assets/795966a0-d3ef-4f3b-9485-fe8bbaa3a449)
 
Arduino Mega 2560: Is a microcontroller board based on the ATmega2560. It has 54 digital input/output pins and 16 analog inputs, a 16 MHz crystal oscillator, a USB connection, a power jack, an ICSP header, and a reset button. The arduino is the board that contains the code that allow us to accomplish the challenge, using the sensors data to make the necessary movements.

**ultrasonic sensors (hc-sr04):**
| Specifications: |
| ------------- |
| Accuracy: 3cm |
| Measurement Range: 2cm to 400cm |
| Resolution: 0.3 cm |
| Frequency: 40 kHz|
| Input voltage: 5V  |
| Operating Current: 15 mA |
![Ultrasonic_Sensor](https://github.com/user-attachments/assets/9172b512-f920-4b1e-98cb-fac84d70ee8a)


It is a sensor that uses ultrasonic sounds to detect the bounce time of sound from one side to the other. Using the Arduino Mega 2560 we can determine the distance based on the time it takes for the wave to return, performing the function of determining when there is a wall nearby, and thus making the corresponding turn

**Pixy Cam v2:**
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

The camera is capable of detecting seven colors simultaneously and It is equipped with an internal processor, which lets us explore just the necessary information for the Arduino to evade in the necessary way, depending on the obstacle colour

**Battery:**
| Specifications: |
| ------------- |
| Capacity: 9900 mAh |
| Voltage: 3.6 V o 3.7 V |
| Discharge rate:  0.25C |
| Weight: 45 g (one battery) |
| Size: 18 mm diameter, 65 mm length |
![gtf](https://github.com/user-attachments/assets/c771bb60-3599-4566-bee3-3cf6875fe9cd)

This are the batteries tha we are gonna use in the vehicle that are li-ion batteries









