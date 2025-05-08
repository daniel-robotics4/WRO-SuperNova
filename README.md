future engenieers
===
Our project consists of creating an automated land vehicle capable of navigating an environment marked by colored obstacles. The vehicle takes different paths depending on the color of the obstacle. We use an Arduino-based program (C++) that includes code for ultrasonic sensors, which can detect objects at a predetermined distance. When an obstacle is detected, the sensors send a signal to the Arduino circuit board, which then directs the vehicle’s movement system to turn either clockwise or counterclockwise, as pre-established.Additionally, the vehicle is equipped with a camera module (name of the camera) to detect colored obstacles and avoid them based on their color
the code have different rutines depending on :
The code has different routines for:

Movement execution, using a servo motor that can position itself anywhere within its range of motion and from there execute a direction.

Obstacle detection, through ultrasonic sensors that determine the proximity of obstacles and send a signal to the board, which will activate the color sensor.

Color identification using a color sensor. Depending on the identified color, the vehicle will respond as follows: if it is red, it will go to the right; if green, to the left; if black, it will recognize it as a wall and avoid it by turning clockwise or counterclockwise according to the challenge requirements.

Stopping upon detecting the color magenta and parking in the designated area for that purpose.




