#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo myservo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Starting...\n");
  pixy.init();
  myservo.attach(9);
  myservo.write(90);
}

void loop() {
 delay (1000);
 int i; 
  
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks) {
    if (pixy.ccc.blocks[i].m_x > 135 && pixy.ccc.blocks[i].m_x < 180) {
    Serial.print("Middle ");
    myservo.write(90);
    delay (1000);
    }
    if (pixy.ccc.blocks[i].m_x>180) {
    Serial.print("Right ");
    myservo.write(0);
    delay (1000);
    }
    if (pixy.ccc.blocks[i].m_x<135) {
    Serial.print("Left ");
    myservo.write(180);
    delay (1000);
    }
  }
  for (i=0; i<pixy.ccc.numBlocks; i++){
  Serial.print(" block ");
  Serial.print(i);
  Serial.print(": ");
  pixy.ccc.blocks[i].print();
    }
  }