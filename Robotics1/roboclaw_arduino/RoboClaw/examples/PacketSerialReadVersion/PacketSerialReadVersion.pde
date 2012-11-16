#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

BMSerial terminalSerial(0,1);
RoboClaw rc(5,6);

void setup() {
  terminalSerial.begin(38400);
  rc.begin(2400);
}

void loop() {
  char version[32];

  if(rc.ReadVersion(address,version)){
    terminalSerial.println(version);  
  }

  delay(100);
}

