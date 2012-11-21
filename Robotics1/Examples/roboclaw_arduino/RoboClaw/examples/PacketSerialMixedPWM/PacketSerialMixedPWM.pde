#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

RoboClaw roboclaw(5,6);

void setup() {
  roboclaw.begin(2400);
}

void loop() {
  roboclaw.ForwardMixed(address, 64);
  delay(2000);
  roboclaw.BackwardMixed(address, 64);
  delay(2000);
  roboclaw.TurnRightMixed(address, 64);
  delay(2000);
  roboclaw.TurnLeftMixed(address, 64);
  delay(2000);
  roboclaw.ForwardBackwardMixed(address, 32);
  delay(2000);
  roboclaw.ForwardBackwardMixed(address, 96);
  delay(2000);
  roboclaw.LeftRightMixed(address, 32);
  delay(2000);
  roboclaw.LeftRightMixed(address, 96);
  delay(2000);
}
