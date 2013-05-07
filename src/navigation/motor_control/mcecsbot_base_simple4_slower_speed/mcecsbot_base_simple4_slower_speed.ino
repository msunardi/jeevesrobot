

/*  This program is designed to run on an Arduino Uno with 2 basic micro Roboclaw motor controllers connected, both running in packet serial mode.
 It recieves commands from a serial port in a specified format, that is: fixed length, byte0 = 0xFF (dedicated), byte1 = 0xFF (dedicated), byte2 = opcode, 
 bytes3-7 = motor command data.  This format is described in more detail in Jeramy's homework 3 found on the MCECS bot project page.
 Version: 0.2
 */
#include "Wire.h" 
#include "BMSerial.h" 
#include "RoboClaw.h" 

// define PID constants on roboclaw for speed/distance commands
#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define qpps 44000
#define sonar_controller 2
#define STOP 4
#define FORWARD 2
#define BACKWARD 3
#define RIGHT 0
#define LEFT 1

BMSerial terminalSerial(0,1);          // TerminalSerial is the debug serial windows when 0,1 is selected
//RoboClaw roboclaw(50,52);              // 5,6 represent the pins on the arduino that the motor controller is connected to
RoboClaw roboclaw(46,48);              // 5,6 represent the pins on the arduino that the motor controller is connected to

    uint8_t MotorSpeed;                // Holds the current motorspeed of the robot
    uint8_t TopMotorSpeed = 30;        // Top speed is actually TopMotorSpeed + 1
    uint8_t NoMotorSpeed = 0;          // variable to name a speed of zero
    uint8_t increment = 5;             // Rate of acceleration. Make sure it is a multiple of TopMotorSpeed
    boolean stopmoving = false;        // Tells whether the robot has been asked to stop moving

    boolean first_iteration = false;   // As the robot accelerates it only needs to send the Uno (sonar_controller)
                                       // when it initiates a new movement. Otherwise the controller would be bogged
                                       // down with unnecessary communication.   

    boolean was_interrupt = false;     // If a sonar detects and object that is too close it drives a pin high. This
                                       // variable stores the value of that interrupt pin (currently enacted as an 
                                       // interrupt. The base code simply polls the "interrupt" pin).

    int threshold = 15;                // Threshold. Holds the distance at which a sonar will generate an "interrupt" 
                                       // Threshold must be the same value on the Uno(sonar_controller).    

    int available_movements = 4;       // This is the number of movements available to the robot. Currently, 
                                       // there is only forward, backward, rotate left, and rotate right. If you add
                                       // any movement increment this by one. It is used to make sure nothing happens
                                       // if incorrect input values are received. 

    int new_movement = 5;              // Holds the command value from any input device telling what movement to make. 
    int old_movement = 6;              // Holds the last value of new_movement. Used to transition out of an old movement
                                       // and into a new movement. 

    int update;                        // Variable to hold an intermediate value. If robot is accelerating/decelerating this
                                       // is used to compare against new/old movements. 

    int interruptPin = 2;              // Pin used to signal an obstacle has been detected by sonars. Logic High. 

    void (*function[4])(uint8_t) = {&rotateRight, &rotateLeft, &goForward, &goBackward};  // An array of pointers to movement functions. 

void setup() {
  pinMode(interruptPin, INPUT);  // Set Mega pin 2 to input. 
  roboclaw.begin(2400);          // Baud rate of connection to motor driver
  pinMode(22, OUTPUT);           // not sure? 
  Serial.begin(9600);            // serial to pc
  Wire.begin();

  // instantiate PID constants
  roboclaw.SetM1Constants(0x80,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x80,Kd,Kp,Ki,qpps); 
  roboclaw.SetM1Constants(0x81,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x81,Kd,Kp,Ki,qpps);  
}

///////////////       Main Loop        ///////////////////

void loop() { 

    if(Serial.available()){                        // If new movement available, retreive.
    new_movement = Serial.read();      
    }

    was_interrupt = digitalRead(interruptPin);    // Poll interrupt pin from sonar controller. 
   if(was_interrupt == true){
     Serial.println("Interrupt true.");
     stopmoving = true;                           // If pin HIGH then set stopmoving to true. 
     detected_obstacle();                         // jump to stop routine for detected obstacle
     was_interrupt = false;
    }

    switch(new_movement){                         // switch case for debugging using keyboard controls

      case 108:                  // Keyboard l      Right
        new_movement = RIGHT;
        stopmoving = false;                         // Left and Right are reversed for Kinect Control. Because
        break;                                      // the control happens while facing the robot

      case 106:                  // Keyboard j     i Left
        new_movement = LEFT;
        stopmoving = false;
        break;

      case 107:
        new_movement = STOP;        // Keyboard k      Stop
        //detected_obstacle();
        //new_movement = old_movement;
        stopmoving = true;
        new_movement = Stop_old_movement();
        break;

      case 105:                  // Keyboard i      Forward
        new_movement = FORWARD;
        stopmoving = false;
        break;  

      case 44:                   // Keyboard ,      Backward
        new_movement = BACKWARD;
        stopmoving = false;
        break;

        default: 
        break;
    }

    doMove(new_movement);
    /*if(stopmoving == false && new_movement < available_movements){        // If robot was not instructed to stop 
       new_movement = Stop_old_movement();                                // and new_movement is a valid movement
                                                                          // deccelerate from old movement. 

       if(stopmoving == false && new_movement < available_movements){     // If robot was not instructed to stop
         first_iteration = true;                                          // and new_movement is a valid movement
         new_movement = Start_new_movement();                             // set first_iteration to true, signaling
       }                                                                  // the first time in the acceleration loop. 
    }*/

}
/////////////////       End Main Loop            /////////////////////////////

void doMove(int new_move) {
 if(stopmoving == false && new_movement < available_movements){        // If robot was not instructed to stop 
       new_movement = Stop_old_movement();                                // and new_movement is a valid movement
                                                                          // deccelerate from old movement. 

       if(stopmoving == false && new_movement < available_movements){     // If robot was not instructed to stop
         first_iteration = true;                                          // and new_movement is a valid movement
         new_movement = Start_new_movement();                             // set first_iteration to true, signaling
       }                                                                  // the first time in the acceleration loop. 
    } 
}

/////////////////       Stop Old Movement Function       /////////////////////
/*
When we jump into this loop we set update to the value of new_movement. This 
variable allows us to make sure that if we are sent a new movement during this 
loop we have a variable to compare to. We then deccelerate from the old movement
until we reach a speed of 0. At which point we update the old_movement to the 
new_movement. 

If we enter this loop and we have been told that to stop (stopmoving = true)
then new_movement becomes equal to old_movement.

We continually check to make sure we are not receiving new movement data. 
If we do receive new movement data we jump out of Stop_old_movement into the 
main loop.

We always send back new_movement to the main loop in case it has changed. 
*/
int Stop_old_movement(){ 
  update = new_movement;                                   
          while( new_movement != old_movement && MotorSpeed > NoMotorSpeed){
              Serial.println("stopping old movement ...");
              MotorSpeed = MotorSpeed - increment;
              first_iteration = false;
              function[old_movement](MotorSpeed);

              if (MotorSpeed == NoMotorSpeed && stopmoving == true){
                new_movement = old_movement;
                Serial.println("updated kinect value from STOP");
              }
               else if (MotorSpeed == NoMotorSpeed){
              old_movement = new_movement;
              Serial.println("updated kinect value from STOP");
              }

              if(Serial.available()){
                 update = Serial.read();
                 if (update != new_movement && update != 10){
                 new_movement = update;
                 Serial.println(update);
                 Serial.println("Interrupted Stop Movement"); 
                 break;
                 }
              }
          }
return new_movement;
}
/////////////////       End stop Old Movement Function       /////////////////////

/////////////////       Start New Movement Function       /////////////////////
/*
Similar to the stop function the start continually checks for new movement input. 
It accelerates up to the TopMotorSpeed and leaves the loop. first_iteration is 
set to false after the first call of a movement function so that the Mega does not
continue sending movement data to the sonar controller when the movement isn't changing. 
*/

int Start_new_movement(){
   update = new_movement;
      for( MotorSpeed; MotorSpeed <= TopMotorSpeed; MotorSpeed = MotorSpeed + increment){
                function[new_movement](MotorSpeed);
                first_iteration = false;
                if(MotorSpeed == TopMotorSpeed){
                old_movement = new_movement; 
                Serial.println("updated kinect from START");
               }

               if(Serial.available()){
                 update = Serial.read();
                 if (update != new_movement && update != 10){
                 old_movement = new_movement;
                 new_movement = update;
                 Serial.println(update);
                 Serial.println("Interrupted Start Movement"); 
                 break;
                 } 
                }
              }
       
  return new_movement;
}

void rotateRight(uint8_t MotorSpeed){
  Serial.print("Rotate Right:   ");
  Serial.println(MotorSpeed);
  roboclaw.BackwardM1(0x80, MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
  if(first_iteration == true){
    Serial.println("    First Time Right");
    Wire.beginTransmission(sonar_controller);
    Wire.write(new_movement);
    Wire.endTransmission();
  }
   was_interrupt = digitalRead(interruptPin);
   /*if(was_interrupt == true){
     stopmoving = true; 
     detected_obstacle();
    }*/

}

void rotateLeft(uint8_t MotorSpeed){
  Serial.print("Rotate Left:   ");
  Serial.println(MotorSpeed);
  roboclaw.ForwardM1(0x80, MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
  if(first_iteration == true){
    Serial.println("    First Time Left");
    Wire.beginTransmission(sonar_controller);
    Wire.write(new_movement);
    Wire.endTransmission();
  }

      was_interrupt = digitalRead(interruptPin);
    if(was_interrupt == true){
       stopmoving = true; 
       detected_obstacle();
    }
}

void goForward(uint8_t MotorSpeed){
  Serial.print("Forward:   ");
  Serial.println(MotorSpeed);
  roboclaw.ForwardM1(0x80,MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  //roboclaw.ForwardM2(0x80,15);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
  //roboclaw.ForwardM2(0x81,15);
  if(first_iteration == true){
    Serial.println("   First Time Forward");
    Wire.beginTransmission(sonar_controller);
    Wire.write(new_movement);
    Wire.endTransmission();
  }

      was_interrupt = digitalRead(interruptPin);
    if(was_interrupt == true){
       stopmoving = true;
       detected_obstacle();
    }
}

void goBackward(uint8_t MotorSpeed){
  Serial.print("Backward:   ");
  Serial.println(MotorSpeed);
  roboclaw.BackwardM1(0x80,MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
  if(first_iteration == true){
    Serial.print("   First Backward");
    Wire.beginTransmission(sonar_controller);
    Wire.write(new_movement);
    Wire.endTransmission();
  }

    was_interrupt = digitalRead(interruptPin);
    if(was_interrupt == true){
      stopmoving = true; 
      detected_obstacle();

    }
}

void goStop(){
  roboclaw.ForwardM1(0x80,0);
  roboclaw.ForwardM2(0x80,0);
  roboclaw.ForwardM1(0x81,0);
  roboclaw.ForwardM2(0x81,0);
  new_movement = old_movement;
  MotorSpeed = 0;
}

// Detected Obstacle Function //
/* 
This function runs when the sonar controller sets 
the Mega pin 2 HIGH. Indicated that an obstacle is 
in the way of the robot. The Uno will send which 
sonar detected the obstacle and how far the obstacle 
is from that sonar. If the obstacle is less than 10 inches
away it will perform an emergency stop where the robot
stops immediately. If 10inches < obstacle < 15inches the
the robot will decelerate. This function will get stuck
in the while loop if the obstacle remains, thereby 
preventing the robot from moving until the obstacle
has been removed. 
*/

void detected_obstacle(){
  uint8_t sonar_number, obstacle;
  int interrupted_movement;
  interrupted_movement = new_movement;
  
  
  //char sonar_number, obstacle;
  Wire.requestFrom(sonar_controller, 2);
  int i = 0;
  delay(100);
  Serial.print("Bytes available: ");
  Serial.println(Wire.available());
  /*while(Wire.available())
  {
    byte c = Wire.read();
    Serial.print("From wire: ");
    Serial.println(c);
    //uno[i] = c;
    //obstacle = Wire.read();    
    //Serial.print(" obstacle:    ");
    //Serial.print(obstacle);
    //i = i+1;
  }*/
  /*if(Wire.available())
  {
    sonar_number = Wire.read();
    //Serial.print("From wire: ");
    //Serial.println(c); 
    Serial.print(" sonar number:  "); 
    Serial.println(sonar_number);
  }*/
   delay(100);
   obstacle = Wire.read();
   delay(100);
   sonar_number = Wire.read();
   Serial.print(" obstacle:    ");
   Serial.print(obstacle);
   Serial.print(" sonar number:  "); 
   Serial.println(sonar_number);
    if(obstacle < 20){
     Serial.println("obstacle < 10"); 
      //goStop();
      new_movement = RIGHT;
      stopmoving = false; 
      doMove(new_movement);
      //decide(obstacle, sonar_number);
      
      } else{
        //new_movement = Stop_old_movement();
      }
   //delayMicroseconds(250);
   int count=0;
   while((obstacle < threshold) && (sonar_number > 0)){
     Serial.println("waiting until no more obstacle");
     Wire.requestFrom(sonar_controller, 2);
    
     obstacle = Wire.read();
     sonar_number = Wire.read();
     Serial.write(sonar_number);
     //Serial.write("  ");
     //Serial.write(obstacle);
     delayMicroseconds(250);
     //
     decide(obstacle, sonar_number);
     
     if (count>100) {
       count = 0;
       break;
     } else {
       count = count + 1;
       Serial.println(count);
     }
   }
   new_movement = interrupted_movement;
   stopmoving = false;
   doMove(new_movement);
   Serial.println("end detected_obstacle()");

}

// End Detected Obstacle Function // 

// Decide what to do / where to move when there's an obstacle
void decide(int obstacle, int sonar_number) {
  // Sonars:
  // front left: 1,2,3
  // back left:  4,5,6
  // back right: 7,8,9
  // front right: 10,11,12
  Serial.print("Obstacle: ");
  Serial.println(obstacle);
  Serial.print("Sonar number: ");
  Serial.println(sonar_number);
  Serial.println("Trying to decide what to do...");
  if (sonar_number==7||sonar_number==6||sonar_number==5) {
    Serial.println("Turn Right");
    new_movement=RIGHT;
    //doMove();  // turn right
  } else if (sonar_number==4||sonar_number==8) {
    Serial.println("Go Forward");
    new_movement=FORWARD;
    //doMove(FORWARD);  // go forward
  } else if (sonar_number==12) {
    Serial.println("Turn Left");
    new_movement=LEFT;
    //doMove(LEFT);  // turn left
  } else if (sonar_number==13||sonar_number==7) {
    Serial.println("Go Reverse");
    new_movement=BACKWARD;
    //doMove(BACKWARD);  // go reverse
  } else {
    Serial.println("Stopping...");
    new_movement=STOP;
    //doMove(STOP);
  }
  doMove(new_movement);
}
///////
//
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
