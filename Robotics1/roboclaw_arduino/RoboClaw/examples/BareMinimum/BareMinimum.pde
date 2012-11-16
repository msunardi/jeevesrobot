#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

RoboClaw roboclaw(5,6);

void setup() {
  roboclaw.begin(38400);

  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
