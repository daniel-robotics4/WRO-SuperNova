future engenieers
===
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. The vehicle takes different paths depending on the color of the obstacle. We use an Arduino-based program (C++) that includes code for ultrasonic sensors, which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the Arduino circuit board, which then directs the vehicle’s movement system to turn either clockwise or counterclockwise, as pre-established.Additionally, the vehicle is equipped with a camera module (name of the camera) to detect colored obstacles and avoid them based on their color

The core of the system is based on the Arduino Mega 2560 microcontroller, which offers ample input/output pins and processing power to handle multiple sensors and actuators simultaneously. The vehicle integrates a DC motor equipped with an encoder to provide precise feedback on wheel rotation, enabling closed-loop speed and distance control. Three ultrasonic sensors are strategically mounted to provide comprehensive environmental awareness by measuring distances to obstacles in front and on both sides of the vehicle. A motor driver shield manages power delivery and control signals to the motor, while a servomotor is used to actuate steering,the chasis made for this vehicle was made from scratch in a 3d tool an then printed


## Electronic Components

**DC motor with encoder:**
The propulsion system relies on a brushed DC motor paired with an encoder. The motor provides the mechanical force required to move the vehicle, while the encoder outputs two pulse signals (channels A and B) that indicate the rotation direction and speed by measuring the number of pulses per revolution.

**Arduino mega:**
Arduino Mega 2560: Is a microcontroller board based on the ATmega2560. It has 54 digital input/output pins and 16 analog inputs, a 16 MHz crystal oscillator, a USB connection, a power jack, an ICSP header, and a reset button. The arduino is the board that contains the code that allow us to accomplish the challenge, using the sensors data to make the necessary movements.

**servomotor:**
A servomotor is integrated into the system to control steering. The servo receives PWM signals from a dedicated Arduino Mega pin, allowing precise angular positioning between 0° and 180°. The servo’s position is controlled programmatically to perform smooth and accurate movements.

**ultrasonic sensors (hc-sr04):**
It is a sensor that uses ultrasonic sounds to detect the bounce time of sound from one side to the other. Using the Arduino Mega 2560 we can determine the distance based on the time it takes for the wave to return, performing the function of determining when there is a wall nearby, and thus making the corresponding turn

**Motor driver shield:**
The motor driver shield is designed to handle the power requirements of the DC motor and provide an easy interface for Arduino Mega. It accepts PWM signals for speed control and digital signals for motor direction, enabling forward and reverse movement. The shield includes built-in protection features such as current limiting and thermal shutdown to safeguard components during operation.

**Camera:**
The camera is capable of detecting seven colors simultaneously and It is equipped with an internal processor, which lets us explore just the necessary information for the Arduino to evade in the necessary way, depending on the obstacle colour

**Motor driver shield (L293d)**
is a driver board based on L293 IC, which can drive 4 DC motors and 2 stepper or Servo motors at the same time. Each channel of this module has the maximum current of 1.2A and doesn't work if the voltage is more than 25v or less than 4.5v.

# Rutines #








