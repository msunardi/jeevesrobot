#include "BMSerial.h"
#include "RoboClaw.h"

// define PID constants on roboclaw for speed/distance commands
#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define qpps 44000

BMSerial terminalSerial(0,1); // TerminalSerial is the debug serial windows when 0,1 is selected
RoboClaw roboclaw(50,52); // 5,6 represent the pins on the arduino that the motor controller is connected to

    uint8_t MotorSpeed;
    uint8_t TopMotorSpeed = 30;  //Top speed is actually TopMotorSpeed + 1
    uint8_t NoMotorSpeed = 5;
    uint8_t increment = 6;
    boolean stopmoving = false;
    
    int kinectVal = 4;
    int oldKinectVal = 0;
    int update;
    
    void (*function[4])(uint8_t) = {&rotateRight, &rotateLeft, &goForward, &goBackward};
    
    //uint8_t x, y;

void setup() {
  
    // setup bumper interupt pins
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, bumper_2, FALLING);
  attachInterrupt(1, bumper_3, FALLING);

  //terminalSerial.begin(38400);   // Baud rate of debug serial port
  roboclaw.begin(2400);          // Baud rate of connection to motor driver
  pinMode(22, OUTPUT);
  Serial.begin(9600);            // serial to pc

  // instantiate PID constants
  roboclaw.SetM1Constants(0x80,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x80,Kd,Kp,Ki,qpps); 
  roboclaw.SetM1Constants(0x81,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x81,Kd,Kp,Ki,qpps); 

  
  
}


void loop() { 
    
  
    if(Serial.available()){
    kinectVal = Serial.read();
    }
    
    
    switch(kinectVal){
      
      case 'd':
        kinectVal = 0;
        stopmoving = false;
        
            if(stopmoving == false && kinectVal < 5){
       kinectVal = Stop_old_movement();
       if(stopmoving == false && kinectVal < 5){
       kinectVal = Start_new_movement();
       }
    }
        
        break;
      case 'a':
        kinectVal = 1;
        stopmoving = false;
        
            if(stopmoving == false && kinectVal < 5){
       kinectVal = Stop_old_movement();
       if(stopmoving == false && kinectVal < 5){
       kinectVal = Start_new_movement();
       }
    }
        
        break; 
      case 'x':
        kinectVal = 4;
        //kinectVal = oldKinectVal;
        stopmoving = true;
        kinectVal = Stop_old_movement();
        
            if(stopmoving == false && kinectVal < 5){
       kinectVal = Stop_old_movement();
       if(stopmoving == false && kinectVal < 5){
       kinectVal = Start_new_movement();
       }
    }
        
        break;
      case 'w':
        kinectVal = 2;
        stopmoving = false;
        
            if(stopmoving == false && kinectVal < 5){
       kinectVal = Stop_old_movement();
       if(stopmoving == false && kinectVal < 5){
       kinectVal = Start_new_movement();
       }
    }
        
        break;
      case 's':
        kinectVal = 3;
        stopmoving = false;
        
            if(stopmoving == false && kinectVal < 5){
       kinectVal = Stop_old_movement();
       if(stopmoving == false && kinectVal < 5){
       kinectVal = Start_new_movement();
       }
    }
        
        break;
        
        default: 
        break;
    }
    

}

int Stop_old_movement(){ 
update = kinectVal;
    //Serial.print("pressed: ");
    //Serial.println(kinectVal);
          while( kinectVal != oldKinectVal && MotorSpeed > NoMotorSpeed){
              MotorSpeed = MotorSpeed - increment;
              function[oldKinectVal](MotorSpeed);
              
              if (MotorSpeed == NoMotorSpeed && stopmoving == true){
                kinectVal = oldKinectVal;
                //Serial.println("updated kinect value from STOP");
              }
              if (MotorSpeed == NoMotorSpeed){
              oldKinectVal = kinectVal;
              //Serial.println("updated kinect value from STOP");
              }
      
              if(Serial.available()){
                 update = Serial.read();
                 if (update != kinectVal){
                 kinectVal = update;
                 //Serial.println("Interrupted Stop Movement"); 
                 break;
                 }
              }
          }
return kinectVal;
}

int Start_new_movement(){
    update = kinectVal;
    //Serial.print("pressed: ");
    //Serial.println(kinectVal);
      for( MotorSpeed; MotorSpeed <= TopMotorSpeed; MotorSpeed = MotorSpeed + increment){
        
                function[kinectVal](MotorSpeed);
                
                if(MotorSpeed == TopMotorSpeed){
                oldKinectVal = kinectVal; 
                //Serial.println("updated kinect from START");
               }
               
               if(Serial.available()){
                 update = Serial.read();
                 if (update != kinectVal){
                 oldKinectVal = kinectVal;
                 kinectVal = update;
                 //Serial.println("Interrupted Start Movement"); 
                 break;
                 } 
                }
              }
  return kinectVal;
}

void rotateRight(uint8_t MotorSpeed){
  //Serial.print("Rotate Right:   ");
  //Serial.println(MotorSpeed);
  roboclaw.BackwardM1(0x80, MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
}

void rotateLeft(uint8_t MotorSpeed){
  //Serial.print("Rotate Left:   ");
  //Serial.println(MotorSpeed);
  roboclaw.ForwardM1(0x80, MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
}


void strafeLeft(uint8_t MotorSpeed){
  roboclaw.ForwardM1(0x80, MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
}


void strafeRight(uint8_t MotorSpeed){
  roboclaw.BackwardM1(0x80,MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
}


void goForward(uint8_t MotorSpeed){
  //Serial.print("Forward:   ");
  //Serial.println(MotorSpeed);
  roboclaw.ForwardM1(0x80,MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
}

void goBackward(uint8_t MotorSpeed){
  //Serial.print("Backward:   ");
  //Serial.println(MotorSpeed);
  roboclaw.BackwardM1(0x80,MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
}


void goStop(){
  roboclaw.ForwardM1(0x80,0);
  roboclaw.ForwardM2(0x80,0);
  roboclaw.ForwardM1(0x81,0);
  roboclaw.ForwardM2(0x81,0);
}


/*
void displayspeed(void)
{
  uint8_t status;
  bool valid;

  uint32_t enc1 = roboclaw.ReadEncM1(0x80, &status, &valid);
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
*/

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
