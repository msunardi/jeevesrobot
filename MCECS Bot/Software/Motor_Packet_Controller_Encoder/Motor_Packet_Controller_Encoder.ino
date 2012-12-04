#include "BMSerial.h"
#include "RoboClaw.h"

#define address 0x80

#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define qpps 44000

//TerminalSerial is the debug serial windows when 0,1 is selected
//BMSerial terminalSerial(0,1);
//RoboClaw rc(5,6);

//5,6 represent the pins on the arduino that the motor controller is connected to
RoboClaw roboclaw(5,6);

int ab = 0;

void setup() {
  //Baud rate of debug serial port
  //terminalSerial.begin(38400);
  //rc.begin(2400);
  //Baud rate of connection to motor driver
  roboclaw.begin(2400);
  Serial.begin(9600);
  
  //pinMode(22, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(2, bumper_2, LOW);
  attachInterrupt(3, bumper_3, LOW);

  //roboclaw.ForwardM1(address,0);

  //serial to pc
  
  
  roboclaw.SetM1Constants(0x80,Kd,Kp,Ki,qpps);

}

void loop() {
  //char version[32];
  uint8_t encoder = 0;
  byte bytein = 0;
  int in = 0;
  
  
  if(Serial.available() > 0)
  {
    bytein = Serial.read();
    switch (ab){
      case 0:
         roboclaw.BackwardM1(0x80,bytein);
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
  //if(){
  //if(roboclaw.ReadVersion(0x80,version)){
    uint32_t enc1;
    enc1 = roboclaw.ReadEncM1(0x80,&encoder);
    //Serial.println(enc1,DEC);
    
  //}
  
  delay(100);
}

void bumper_2() 
{
  Serial.println("Bumper2");
}

void bumper_3() 
{
  Serial.println("Bumper3");
}


