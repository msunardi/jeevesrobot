/*  This program is designed to run on an Arduino Uno with 2 basic micro Roboclaw motor controllers connected, both running in packet serial mode.
    It recieves commands from a serial port in a specified format, that is: fixed length, byte0 = 0xFF (dedicated), byte1 = 0xFF (dedicated), byte2 = opcode, 
    bytes3-7 = motor command data.  This format is described in more detail in Jeramy's homework 3 found on the MCECS bot project page.
    Version: 0.1  NOT TESTED MAY HAVE ERRORS!!!
*/

#include "BMSerial.h"
#include "RoboClaw.h"

// TerminalSerial is the debug serial windows when 0,1 is selected
BMSerial terminalSerial(0,1);

// 5,6 represent the pins on the arduino that the motor controller is connected to
RoboClaw roboclaw(5,6);

int sof = 0;      // first half of start of frame
int data = 0;     // reading data or not
int datarray[6];  // initialize data array

void setup() {
  // Baud rate of debug serial port
  terminalSerial.begin(38400);

  // Baud rate of connection to motor driver
  roboclaw.begin(2400);
  
  // ???
  pinMode(22, OUTPUT);

  // serial to pc
  Serial.begin(9600);

}

void loop() {
  
  // initialize first two data bytes to be read in
  byte bytein0 = 0;  
  byte bytein1 = 0;
  
  // initialize motor direction bits to be filled by the direction vector
  byte M1dir = 1;
  byte M2dir = 1;
  byte M3dir = 1;
  byte M4dir = 1;
  
  // when serial data is available read in the first two bytes
  if(Serial.available() > 0)
  { 
    bytein0 = Serial.read();
    delay(100);  // this delay is apparently required to receive the data
    bytein1 = Serial.read();
    
    // if the first two bytes are the designated start of frame, 0xFFFF, then read in the data.  This needs to be a unique value w.r.t. the data
    if(bytein0 == 0xFF && bytein1 == 0xFF)  
    {
      // this loop reads each byte of the fixed length (6 bytes) message from the PC program into an array, after the start of frame header.  
      // Byte0 = opcode, Byte1-5 = motor command data. It may need to require that data is available, but it seems to work well as is.
      for(int i = 0; i < 6; i++) {
      datarray[i] = Serial.read();
      delay(100);
      }    
      
      // read direction vector into individual bits      
      M1dir = bitRead(datarray[1],0);
      M2dir = bitRead(datarray[1],1);
      M3dir = bitRead(datarray[1],2);
      M4dir = bitRead(datarray[1],3);
      
      // print all of the data received
      Serial.print("opcode: ");
      Serial.println(datarray[0], DEC);
      Serial.print("M1dir: ");
      Serial.println(M1dir);
      Serial.print("M2dir: ");
      Serial.println(M2dir);
      Serial.print("M3dir: ");
      Serial.println(M3dir);
      Serial.print("M4dir: ");
      Serial.println(M4dir);
      Serial.print("Motor1 speed: ");
      Serial.println(datarray[2], DEC);
      Serial.print("Motor2 speed: ");
      Serial.println(datarray[3], DEC);
      Serial.print("Motor3 speed: ");
      Serial.println(datarray[4], DEC);
      Serial.print("Motor4 speed: ");
      Serial.println(datarray[5], DEC);
      

      
      /*
      //then apply the data to the roboclaw
      switch (datarray[0]) {
        case 0:
          // motor 1 command depending on direction value
          if(M1dir=1)
          {
          roboclaw.ForwardM1(0x81,datarray[2]);
          }
          else if(M1dir=0)
          {
          roboclaw.BackwardM1(0x81,datarray[2]);
          }
          
          // motor 2 command depending on direction value
          if(M2dir=1)
          {
          roboclaw.ForwardM2(0x81,datarray[3]);
          }
          else if(M2dir=0)
          {
          roboclaw.BackwardM2(0x81,datarray[3]);
          }          
          
          break;
      }
      */    
    }
    else  //the message was only partialy received, resend
    {
      Serial.println("error: start of frame not received, please resend."); 
    }
        
    //roboclaw.ForwardM1(0x81,bytein);
    //Serial.println("read a\n");     
    //Serial.println(bytein,DEC);
      
    //roboclaw.ForwardM1(0x80,bytein);
    //Serial.println("read b\n");     
    //Serial.println(bytein,DEC);
  
    //Serial.println("read\n");     
    //Serial.println(bytein,DEC);
    //roboclaw.ForwardM1(address,bytein);

    //terminalSerial.println("Hello");

    //if(rc.ReadVersion(address,version)){
    //terminalSerial.println(version);
    
    //}

    delay(100);
    }
}


