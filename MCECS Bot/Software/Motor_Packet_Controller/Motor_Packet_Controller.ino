/*  This program is designed to run on an Arduino Uno with 2 basic micro Roboclaw motor controllers connected, both running in packet serial mode.
 It recieves commands from a serial port in a specified format, that is: fixed length, byte0 = 0xFF (dedicated), byte1 = 0xFF (dedicated), byte2 = opcode, 
 bytes3-7 = motor command data.  This format is described in more detail in Jeramy's homework 3 found on the MCECS bot project page.
 Version: 0.2
 */

#include "BMSerial.h"
#include "RoboClaw.h"

// define PID constants on roboclaw for speed/distance commands
#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define qpps 44000

// TerminalSerial is the debug serial windows when 0,1 is selected
BMSerial terminalSerial(0,1);

// 5,6 represent the pins on the arduino that the motor controller is connected to
RoboClaw roboclaw(5,6);

int sof = 0;      // first half of start of frame
int data = 0;     // reading data or not
int datarray[6];  // initialize data array

void setup() {
  // Baud rate of debug serial port
  //terminalSerial.begin(38400);

  // Baud rate of connection to motor driver
  roboclaw.begin(2400);

  // serial to pc
  Serial.begin(9600);
  
  // setup bumper interupt pins
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, bumper_2, FALLING);
  attachInterrupt(1, bumper_3, FALLING);

  // instantiate PID constants
  roboclaw.SetM1Constants(0x80,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x80,Kd,Kp,Ki,qpps); 
  roboclaw.SetM1Constants(0x81,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x81,Kd,Kp,Ki,qpps); 
}

// function to display encoder information during opcode 1 commands, from roboclaw encoder example
void displayspeed(void)
{
  uint8_t status;
  bool valid;

  uint32_t enc1= roboclaw.ReadEncM1(0x80, &status, &valid);
  if(valid){
    Serial.print("Encoder1:");
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status,HEX);
    Serial.print(" ");
  }
  uint32_t enc2 = roboclaw.ReadEncM1(0x81, &status, &valid);
  if(valid){
    Serial.print("Encoder2:");
    Serial.print(enc2,DEC);
    Serial.print(" ");
    Serial.print(status,HEX);
    Serial.print(" ");
  }
  uint32_t speed1 = roboclaw.ReadSpeedM1(0x80, &status, &valid);
  if(valid){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }
  uint32_t speed2 = roboclaw.ReadSpeedM1(0x81, &status, &valid);
  if(valid){
    Serial.print("Speed2:");
    Serial.print(speed2,DEC);
    Serial.print(" ");
  }
  Serial.println();
}

void loop() {

  // initialize first two data bytes to be read in
  byte bytein0 = 0;  
  byte bytein1 = 0;

  // initialize motor direction bits to be filled by the direction vector for case 0 (opcode 0 is basic motor control)
  byte M1dir;
  byte M2dir;
  byte M3dir;
  byte M4dir;


  // when serial data is available read in the first two bytes
  if(Serial.available() > 0)
  { 
    bytein0 = Serial.read();
    delay(100);  // this delay is apparently required to receive the data
    bytein1 = Serial.read();

    // if the first two bytes are the designated start of frame, 0xFFFF, then read in the data.  This needs to be a unique value w.r.t. t
    // e data
    if(bytein0 == 0xFF && bytein1 == 0xFF)  
    {
      // this loop reads each byte of the fixed length (6 bytes) message from the PC program into an array, after the start of frame header

      // Byte0 = opcode, Byte1-5 = motor command data. It may need to require that data is available, but it seems to work well as is.

      datarray[0] = Serial.read();
      delay(100);


      //then apply the data to the roboclaw
      switch (datarray[0]) {

        // case for opcode 0, raw motor drive commands  
      case 0:

        // read in the data for opcode 1, 5 bytes after opcode
        for(int i = 1; i < 6; i++) {
          datarray[i] = Serial.read();
          delay(100);
        } 

        // read direction vector into individual bits      
        M1dir = bitRead(datarray[1],0);
        M2dir = bitRead(datarray[1],1);
        M3dir = bitRead(datarray[1],2);
        M4dir = bitRead(datarray[1],3);

        /*
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
        */

        // motor 1 command depending on direction value
        if(M1dir==1)
        {
          roboclaw.ForwardM1(0x80,datarray[2]);
        }
        else if(M1dir==0)
        {
          roboclaw.BackwardM1(0x80,datarray[2]);
        }

        // motor 2 command depending on direction value
        if(M2dir==1)
        {
          roboclaw.ForwardM2(0x80,datarray[3]);
        }
        else if(M2dir==0)
        {
          roboclaw.BackwardM2(0x80,datarray[3]);
        } 
        // motor 3 command depending on direction value
        if(M3dir==1)
        {
          roboclaw.ForwardM1(0x81,datarray[4]);
        }
        else if(M3dir==0)
        {
          roboclaw.BackwardM1(0x81,datarray[4]);
        }

        // motor 4 command depending on direction value
        if(M4dir==1)
        {
          roboclaw.ForwardM2(0x81,datarray[5]);
        }
        else if(M4dir==0)
        {
          roboclaw.BackwardM2(0x81,datarray[5]);
        }          

        break;

        // case for opcode 1, encoder speed and distance commands
      case 1:

        // read in data bytes for distance, 4 after opcode...
        for(int i = 1; i < 5; i++) {
          datarray[i] = Serial.read();
          delay(100);
        }

        // then read in characters for speed, 4 more
        char M1speed = Serial.read();
        delay(100);
        char M2speed = Serial.read();
        delay(100);
        char M3speed = Serial.read();
        delay(100);
        char M4speed = Serial.read();
        delay(100);

        /*
        Serial.print("M1speed = ");
        Serial.println(M1speed,DEC);
        Serial.print("M2speed = ");
        Serial.println(M2speed,DEC);        
        Serial.print("M3speed = ");
        Serial.println(M3speed,DEC);
        Serial.print("M4speed = ");
        Serial.println(M4speed,DEC);
        Serial.print("M1dist = ");
        Serial.println(datarray[1]);
        Serial.print("M2dist = ");
        Serial.println(datarray[2]);        
        Serial.print("M3dist = ");
        Serial.println(datarray[3]);
        Serial.print("M4dist = ");
        Serial.println(datarray[4]);
        */

        /* Opcode 1 motor commands are recieved in the format speed[cm/s] and distance[m].  speed is limited to -128 to 127cm/s, distance is limited 
           to 255m.  The mecanum wheels are 8in in diameter, so the circumference is 8PI, which results in the scale factor of 2303 encoder cts/m,
           and 23cts/cm.  This was determined empirically by observing the relation of 1470cts/revolution.
        */
        // motor 1 command
        roboclaw.SpeedDistanceM1(0x80,M1speed*23,datarray[1]*2303);
        //displayspeed();
        //delay(100);

        // motor 2 command
        roboclaw.SpeedDistanceM2(0x80,M2speed*23,datarray[2]*2303);
        //displayspeed();
        //delay(100);

        // motor 3 command
        roboclaw.SpeedDistanceM1(0x81,M3speed*23,datarray[3]*2303);
        //displayspeed();
        //delay(100);

        // motor 4 command
        roboclaw.SpeedDistanceM2(0x81,M4speed*23,datarray[4]*2303);
        //displayspeed();
        //delay(100);        
        
        break;
      }

    }
    else  //the message was only partialy received, resend
    {
      Serial.println("error: start of frame not received, please resend."); 
    }

    delay(100);
  }
}

void bumper_2() 
{
  roboclaw.ForwardM1(0x80,0);
  roboclaw.ForwardM2(0x80,0);
  roboclaw.ForwardM1(0x81,0);
  roboclaw.ForwardM2(0x81,0);
  //Serial.println("b2");
  //delay(25);
}

void bumper_3() 
{
  roboclaw.ForwardM1(0x80,0);
  roboclaw.ForwardM2(0x80,0);
  roboclaw.ForwardM1(0x81,0);
  roboclaw.ForwardM2(0x81,0);
  //Serial.println("b3");
  //delay(25);
}



