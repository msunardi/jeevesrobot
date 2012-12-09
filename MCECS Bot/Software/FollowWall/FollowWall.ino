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
RoboClaw roboclaw(50,52);

int sof = 0;      // first half of start of frame
int data = 0;     // reading data or not
int datarray[9];  // initialize data array

void setup() {
  // Baud rate of debug serial port
  terminalSerial.begin(38400);

  // Baud rate of connection to motor driver
  roboclaw.begin(2400);

  // ???
  pinMode(22, OUTPUT);

  // serial to pc
  Serial.begin(9600);

  // instantiate PID constants
  roboclaw.SetM1Constants(0x80,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x80,Kd,Kp,Ki,qpps); 
  roboclaw.SetM1Constants(0x81,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x81,Kd,Kp,Ki,qpps); 
}

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
  byte byteinDAT = 0;
  // initialize motor direction bits to be filled by the direction vector for case 0 (opcode 0 is basic motor control)
  byte M1dir;
  byte M2dir;
  byte M3dir;
  byte M4dir;

   if(Serial.available() > 0){ 
    bytein0 = Serial.parseInt();
    //delay(100);  // this delay is apparently required to receive the data
    bytein1 = Serial.parseInt();
    for(int i = 0; i <= 8; i++){
    byteinDAT = Serial.parseInt();
    datarray[i] = byteinDAT;
    }

  // when serial data is available read in the first two bytes 
    Serial.print(bytein0, HEX);
    Serial.println(bytein1, HEX);
     
    // if the first two bytes are the designated start of frame, 0xFFFF, then read in the data.  This needs to be a unique value w.r.t. t
    // e data
    if(bytein0 == 0xFF && bytein1 == 0xFF)  
    {

      switch (datarray[0]) {
        
      case 0:
        
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

        // motor 1 command depending on direction value
        if(M1dir==1){
          roboclaw.ForwardM1(0x80,datarray[2]);
        }
        else if(M1dir==0){
          roboclaw.BackwardM1(0x80,datarray[2]);
        }

        // motor 2 command depending on direction value
        if(M2dir==1){
          roboclaw.ForwardM2(0x80,datarray[3]);
        }
        else if(M2dir==0){
          roboclaw.BackwardM2(0x80,datarray[3]);
        } 
        // motor 3 command depending on direction value
        if(M3dir==1){
          roboclaw.ForwardM1(0x81,datarray[2]);
        }
        
        else if(M3dir==0){
          roboclaw.BackwardM1(0x81,datarray[2]);
        }

        // motor 4 command depending on direction value
        if(M4dir==1){
          roboclaw.ForwardM2(0x81,datarray[3]);
        }
        else if(M4dir==0){
          roboclaw.BackwardM2(0x81,datarray[3]);
        }          

        break;

     case 1:
     break;
     
     case 2: 
        byte TopSpeed = datarray[2];
        int wall = datarray[3];
       

    }
    }
    else  //the message was only partialy received, resend
    {
      Serial.println("error: start of frame not received, please resend."); 
    }

    delay(100);
  }
  
  int forever = 1;
  int arraysize = 5;
  int rangevalue[] = { 0, 0, 0, 0, 0};
  
  int sonar6, sonar3, sonar1, sonar12, pulse6, pulse3, pulse1, pulse12;
  sonar6 = 7; 
  sonar3 = 4;
  sonar1 = 2;
  sonar12 = 13;
  boolean rightWall;
  
  while(forever == 1){
    
      for(int i = 0 ; i < arraysize; i ++){
          pulse6 = pulseIn(sonar6, HIGH);
          delay(48);
          rangevalue[i] = pulse6/147;
      }
      
        isort(rangevalue,arraysize);
        pulse6 = mode(rangevalue,arraysize);
        Serial.print("pulse6:  ");
        Serial.print(pulse6);
      
      for(int i = 0 ; i < arraysize; i ++){
          pulse3 = pulseIn(sonar3, HIGH);
          delay(48);
          rangevalue[i] = pulse3/147;
      }
      isort(rangevalue,arraysize);
      pulse3 = mode(rangevalue,arraysize);
      Serial.print("pulse3:  ");
      Serial.println(pulse3);
      Serial.println();
      
      for(int i = 0 ; i < arraysize; i ++){
          pulse1 = pulseIn(sonar1, HIGH);
          delay(48);
          rangevalue[i] = pulse1/147;
      }
      
        isort(rangevalue,arraysize);
        pulse1 = mode(rangevalue,arraysize);
        Serial.print("pulse1:  ");
        Serial.print(pulse1);
      
      for(int i = 0 ; i < arraysize; i ++){
          pulse12 = pulseIn(sonar12, HIGH);
          delay(48);
          rangevalue[i] = pulse12/147;
      }
      isort(rangevalue,arraysize);
      pulse3 = mode(rangevalue,arraysize);
      Serial.print("pulse12:  ");
      Serial.println(pulse12);
      Serial.println();
      
      
          if (pulse3 >= 20 && pulse6 >= 20 && rightWall == true){
            roboclaw.ForwardM1(0x80,30);
            roboclaw.BackwardM2(0x80,30);
            roboclaw.BackwardM1(0x81,30);
            roboclaw.ForwardM2(0x81,30);
            rightWall = true; 
          }
          
          else if(pulse3 <= 10 && pulse6 <= 10 && rightWall == true){
            roboclaw.BackwardM1(0x80,30);
            roboclaw.ForwardM2(0x80,30);
            roboclaw.ForwardM1(0x81,30);
            roboclaw.BackwardM2(0x81,30);
            rightWall = true;
          }
          
          else if( 10 <= pulse3 <= 20 && 10 <= pulse6 <= 20 && rightWall == true ){
            roboclaw.ForwardM1(0x80,30);
            roboclaw.ForwardM2(0x80,30);
            roboclaw.ForwardM1(0x81,30);
            roboclaw.ForwardM2(0x81,30);
            rightWall = true;
          }
          
          else if( pulse1 <= 10 || pulse12 <= 10){
            roboclaw.BackwardM1(0x80,30);
            roboclaw.ForwardM2(0x80,30);
            roboclaw.ForwardM1(0x81,30);
            roboclaw.BackwardM2(0x81,30);
            rightWall = false;
          }
           else if( pulse1 >= 10 || pulse12 >= 10 && rightWall == false){
            rightWall = true;
         }
     }
 

  

void isort(int *a, int n){
// *a is an array pointer function
  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}
//Mode function, returning the mode or median.
int mode(int *x,int n){
  int i = 0;
  int count = 0;
  int maxCount = 0;
  int mode = 0;
  int bimodal;
  int prevCount = 0;
  while(i<(n-1)){
    prevCount=count;
    count=0;
    while(x[i] == x[i+1]){
      count++;
      i++;
    }
    if(count>prevCount&count>maxCount){
      mode=x[i];
      maxCount=count;
      bimodal=0;
    }
    if(count==0){
      i++;
    }
    if(count==maxCount){//If the dataset has 2 or more modes.
      bimodal=1;
    }
    if(mode==0||bimodal==1){//Return the median if there is no mode.
      mode=x[(n/2)];
    }
    return mode;
  }}




