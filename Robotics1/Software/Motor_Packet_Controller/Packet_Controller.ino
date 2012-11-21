#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

//TerminalSerial is the debug serial windows when 0,1 is selected
BMSerial terminalSerial(0,1);
//RoboClaw rc(5,6);

//5,6 represent the pins on the arduino that the motor controller is connected to
RoboClaw roboclaw(5,6);

int ab = 0;

void setup() {
  //Baud rate of debug serial port
  terminalSerial.begin(38400);
  //rc.begin(2400);
  //Baud rate of connection to motor driver
  roboclaw.begin(2400);
  
  pinMode(22, OUTPUT);

  roboclaw.ForwardM1(address,0);

  //serial to pc
  Serial.begin(9600);

}

void loop() {
  char version[32];
  char bytein = 0;
  int in = 0;
  
  
  if(Serial.available() > 0)
  {
    bytein = Serial.read();
    switch (ab){
      case 0:
         roboclaw.ForwardM1(0x81,bytein);
         Serial.println("read a\n");     
         Serial.println(bytein,DEC);

         ab = 1;
         break;
         
      case 1:
         roboclaw.ForwardM1(0x80,bytein);
         Serial.println("read b\n");     
         Serial.println(bytein,DEC);

         ab = 0;
         break;
         
      default:
         break;
      
    }
    
     //Serial.println("read\n");     
     //Serial.println(bytein,DEC);
     //roboclaw.ForwardM1(address,bytein);
  }
  
  
  
  //terminalSerial.println("Hello");

  //if(rc.ReadVersion(address,version)){
    //terminalSerial.println(version);
    
  //}

  delay(100);
}


