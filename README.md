future engenieers
===
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. The vehicle takes different paths depending on the color of the obstacle. We use an Arduino-based program (C++) that includes code for ultrasonic sensors, which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the Arduino circuit board, which then directs the vehicle’s movement system to turn either clockwise or counterclockwise, as pre-established.Additionally, the vehicle is equipped with a camera module (name of the camera) to detect colored obstacles and avoid them based on their color
the code have different rutines depending on :
The code has different routines for:

The core of the system is based on the Arduino Mega 2560 microcontroller, which offers ample input/output pins and processing power to handle multiple sensors and actuators simultaneously. The vehicle integrates a DC motor equipped with an optical encoder to provide precise feedback on wheel rotation, enabling closed-loop speed and distance control. Three ultrasonic sensors are strategically mounted to provide comprehensive environmental awareness by measuring distances to obstacles in front and on both sides of the vehicle. A motor driver shield manages power delivery and control signals to the motor, while a servomotor is used to actuate steering

## Electronic Components

**DC motor whith encoder**
The propulsion system relies on a brushed DC motor paired with an optical quadrature encoder. The motor provides the mechanical force required to move the vehicle, while the encoder outputs two pulse signals (channels A and B) that indicate the rotation direction and speed by measuring the number of pulses per revolution.

**Arduino mega**
Arduino Mega 2560: Is a microcontroller board based on the ATmega2560. It has 54 digital input/output pins and 16 analog inputs, a 16 MHz crystal oscillator, a USB connection, a power jack, an ICSP header, and a rexset button. The arduino is the board that contains the code that allow us to accomplish the challenge, using the sensors data to make the necessary movements.

**servomotor**
A servomotor is integrated into the system to control steering. The servo receives PWM signals from a dedicated Arduino Mega pin, allowing precise angular positioning between 0° and 180°. The servo’s position is controlled programmatically to perform smooth and accurate movements synchronized with the vehicle’s navigation logic.

**ultrasonic sensors**
It is a sensor that uses ultrasonic sounds to detect the bounce time of sound from one side to the other. Using the Arduino Mega 2560 we can determine the distance based on the time it takes for the wave to return, performing the function of determining when there is a wall nearby, and thus making the corresponding turn



