/* This program accepts one of the following commands from a host program and performs the action shown
   on the PSU MCECS bot, and is intended for use with the MRPT navigation application:
 'l' - returns a 5pt LRF scan, 1 byte for each range in cm, 5 bytes total
 'r' - returns odometry (x,y,phi), 2 bytes for each coordinate in MU, MU, and degrees, 6 bytes total
 'w' - move forward
 'a' - rotate left
 's' - move backward
 'd' - rotate right
 'x' - stop moving
 This is intended to run on an Arduino Mega 2560 controlling two Basic Micro Roboclaw motor controllers 
 (with two motors and two encoders each), as well as a parallax LRF mounted to a HiTec servo motor.  There
 are also bumper switches programmed to send stop commands to the motors when activated.  See the MCECS
 bot wiki for (a lot) more information. https://projects.cecs.pdx.edu/projects/roboticsclub-mcecsbot/wiki
 
 March 2013 by Mitch Barton
 V1.0
 */
#include <Wire.h>
#include <Servo.h> 
// #include <SoftwareSerial.h>  don't include because we are using BMserial and there's a conflict
#include <BMSerial.h>
#include <RoboClaw.h>

int left_led = 3;  //led's to indicate which bumpers is trigered.
int right_led = 4;
int front_led = 6;
int back_led = 5;

int bumper_left = 22;  //button on pin 
int bumper_right = 24;
int bumper_front = 26;
int bumper_back = 28;
// opcode which determines case to execute
char pos;

// outgoing final LRF range measurements
byte lrf_data[5];
byte lrf_data_buf[5];

// positions for this particular servo with this particular mounting 
// based on center
int center = 100;              
int pos1 = center + 20;
int pos2 = center + 10;
int pos3 = center - 10;
int pos4 = center - 20;
int right90 = 170;
int left90 = 30;

// address for motor controller
#define address 0x80

// constants for PID on motor controllers
#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define qpps 44000
//#define STOP 4
#define STOP 6
#define FORWARD 2
#define BACKWARD 3
#define RIGHT 0
#define LEFT 1
#define ST_RIGHT 4
#define ST_LEFT 5
#define arraysize 12

// variables for transformation from encoders to odometry (x, y, phi)
int32_t phi = 0;
int32_t phi1 = 0;
int32_t D1 = 0;
int32_t x1 = 0;
int32_t y1 = 0;
int32_t x = 0;
int32_t y = 0;
uint8_t S1 = 0, S2 = 0, S3 = 0, S4 = 0, S5 = 0, S6 = 0, S7 = 0, S8 = 0, S9 = 0, S10 = 0, S11 = 0, S12 = 0;

// the width of the wheels is 17" which converts to 0.4427 map units, 
// but we've also added the scale factor for converting to map units E-8, 
// as well as converting from radians to degrees
const int32_t w = 44270000*PI/180;

// assign Rx Tx serial comm for the motor controllers to pins 5 and 6
/* IMPORTANT: Not all pins can be used. Usable pins are: Pins: 10, 11, 12, 13,  50, 51, 52, 53,  62, 63, 64, 65, 66, 67, 68, 69
Using pins other than these would cause not being able to read the encoder
See this thread for more info: http://forums.basicmicro.net/robo-claw-f504/can-send-commands-to-roboclaw-but-can-t-receive-data-t9885.html
*/

// create servo object
Servo myservo;                 

BMSerial terminalSerial(0,1);          // TerminalSerial is the debug serial windows when 0,1 is selected
RoboClaw roboclaw(50,52);              // 5,6 represent the pins on the arduino that the motor controller is connected to
//RoboClaw roboclaw(46,48);              // 5,6 represent the pins on the arduino that the motor controller is connected to
// constants for motor control

uint8_t MotorSpeed = 0;            // Holds the current motorspeed of the robot, initialized at 5
uint8_t TopMotorSpeed = 30;        // Top speed is actually TopMotorSpeed + 1
uint8_t NoMotorSpeed = 0;          // variable to name a speed of zero
uint8_t increment = 3;//5;             // Rate of acceleration. Make sure it is a multiple of TopMotorSpeed
boolean stopmoving = false;        // Tells whether the robot has been asked to stop moving
boolean doneAccelerating;
boolean first_iteration = false;   // As the robot accelerates it only needs to send the Uno (sonar_controller)
                                   // when it initiates a new movement. Otherwise the controller would be bogged
                                   // down with unnecessary communication.   

boolean was_interrupt = false;     // If a sonar detects and object that is too close it drives a pin high. This
                                   // variable stores the value of that interrupt pin (currently enacted as an 
                                   // interrupt. The base code simply polls the "interrupt" pin).

int threshold = 15;                // Threshold. Holds the distance at which a sonar will generate an "interrupt" 
                                   // Threshold must be the same value on the Uno(sonar_controller).    

int available_movements = 6;       // This is the number of movements available to the robot. Currently, 
                                   // there is only forward, backward, rotate left, and rotate right. If you add
                                   // any movement increment this by one. It is used to make sure nothing happens
                                   // if incorrect input values are received. 
int sonar_controller = 2;
int new_movement = STOP;           // Holds the command value from any input device telling what movement to make. 
int old_movement = STOP;           // Holds the last value of new_movement. Used to transition out of an old movement
                                   // and into a new movement. 

int update;                        // Variable to hold an intermediate value. If robot is accelerating/decelerating this
                                   // is used to compare against new/old movements. 

int interruptPin = 2;              // Pin used to signal an obstacle has been detected by sonars. Logic High
int interruptLED = 7;

//void (*function[4])(uint8_t) = {&rotateRight, &rotateLeft, &goForward, &goBackward};  // An array of pointers to movement functions. 
void (*function[6])(uint8_t) = {&rotateRight, &rotateLeft, &goForward, &goBackward, &strafeRight, &strafeLeft};  // An array of pointers to movement functions. 

int avoid_count = 0;      // how many turns it did to avoid/clear the obstacle
int avoid_direction = 6;  // which direction did it turn to avoid/clear the obstacle
int AVOID_DELAY = 1200;   // arbitrary time delay to turn
uint8_t AVOID_SPEED = 20;
uint8_t AVOID_SPEED_0 = 20;
uint8_t AVOID_SPEED_1 = 13;

// The variables in the following three lines are used in wall_following()
boolean DontMoveLeft = false;
boolean DontMoveRight = false;
uint8_t obstacle1 = 0, obstacle2 = 0, sonar_number1, sonar_number2;

uint8_t sonar_number, obstacle; // used in detected_obstacle() 
//uint8_t sonar1,sonar2,sonar3,sonar4,sonar5,sonar6,sonar7,sonar8,sonar9,sonar10,sonar11,sonar12;
uint8_t sonar[arraysize] = {0,0,0,0,0,0,0,0,0,0,0,0};

boolean DEBUG = false;
boolean statusRequest;


void setup()
{

  // open PC/arduino serial port at 9600 baud
  Serial.begin(9600);

  // open the software serial port to the LRF at 9600 baud
  Serial1.begin(9600);       

  // open the software serial port for the roboclaw
  roboclaw.begin(2400);   
 
  Wire.begin(); 
  
  pinMode(left_led, OUTPUT);  //led output
  pinMode(right_led, OUTPUT);  //led output
  pinMode(front_led, OUTPUT);  //led output
  pinMode(back_led, OUTPUT);  //led output
  
  pinMode(bumper_left, INPUT);  //button input
  pinMode(bumper_right, INPUT);  //button input
  pinMode(bumper_front, INPUT);  //button input
  pinMode(bumper_back, INPUT);  //button input
  
  
  digitalWrite(bumper_left,HIGH);//pull this input on.
  digitalWrite(bumper_right,HIGH);//pull this input on.
  digitalWrite(bumper_front,HIGH);//pull this input on.
  digitalWrite(bumper_back,HIGH);//pull this input on.
  
  // attaches the servo on pin 9 to the servo object
  myservo.attach(8);       

  // initialize the LRF
  delay(2000);
  Serial1.write('U');
  delay(100);
  Serial1.read();

  // set PID constants for motor controller 
  roboclaw.SetM1Constants(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM1Constants(0x81,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(0x81,Kd,Kp,Ki,qpps);  
  
  pinMode(interruptLED, OUTPUT);

  // setup bumper interupt pins
  /*pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, bumper_2, FALLING);
  attachInterrupt(1, bumper_3, FALLING);*/
  
  POST();
  
  doneAccelerating = true;
  statusRequest = true;
}

void loop() {

  readSerial();
  if (pos != 'f')readInterrupt();
  doMove();
  delay(150);
}

void readSerial() {
  
  int temp_cmd = -1;
  
  // if we get a valid byte, write it to our local variable  
  if (Serial.available() > 0) {    
    delay(150);
    //pos = Serial.read();
    //Serial.print("Key: ");
    //Serial.println(pos);
    int prev_movement = new_movement;
    new_movement = Serial.read();
    
    if(new_movement > 40) {
	temp_cmd = keyboardDebug(new_movement);
        if (temp_cmd > -1) { // if temp_cmd is -1, it's not movement related
          new_movement = temp_cmd;
        }
        else {
          new_movement = prev_movement;
        }
    }
  }
}

int keyboardDebug(int pos) {
  
  int debug_pos = -1;  // value=-1 for non-movement commands

  // depending what that byte is, perform an action     
  switch (pos) {
    case 'r':
      // LRF single read
      //simpleread(); 
      complicatedread();
      delay(150);
      debug_pos = -1;      
      break;
      
    case 'm':        
      lrfScan1();
      debug_pos = -1;
      break;       

    // l is the agreed character to start the standard 5pt LRF reading 
    case 'l':
      lrfScan2();
      debug_pos = -1;
      break;

    case 't':
      readEncoder();
      debug_pos = -1;
      break;

    // rotate right command
    case 'd':
      debug_pos = RIGHT;
      /*new_movement = 0;
      stopmoving = false;

      // motion smoothing, return to old motion and slow down, then go to new motion
      if(stopmoving == false && kinectVal < 5){
        kinectVal = Stop_old_movement();
        if(stopmoving == false && kinectVal < 5){
          kinectVal = Start_new_movement();
        }
      }*/

      break;
      
    // rotate left command 
    case 'a':
      debug_pos = LEFT;
      /*kinectVal = 1;
      stopmoving = false;

      // motion smoothing, return to old motion and slow down, then go to new motion
      if(stopmoving == false && kinectVal < 5){
        kinectVal = Stop_old_movement();
        if(stopmoving == false && kinectVal < 5){
          kinectVal = Start_new_movement();
        }
      }*/

      break; 
    
    // stop command  
    case 'x':
      debug_pos = STOP;
      /*kinectVal = 4;
      stopmoving = true;
      kinectVal = Stop_old_movement();

      // motion smoothing, return to old motion and slow down, then go to new motion  
      if(stopmoving == false && kinectVal < 5){
        kinectVal = Stop_old_movement();
        if(stopmoving == false && kinectVal < 5){
          kinectVal = Start_new_movement();
        }
      }*/

      break;
      
    // forward command  
    case 'w':
      debug_pos = FORWARD;
      /*kinectVal = 2;
      stopmoving = false;

      // motion smoothing, return to old motion and slow down, then go to new motion
      if(stopmoving == false && kinectVal < 5){
        kinectVal = Stop_old_movement();
        if(stopmoving == false && kinectVal < 5){
          kinectVal = Start_new_movement();
        }
      }*/

      break;
      
    // backward command  
    case 's':
      debug_pos = BACKWARD;
      /*kinectVal = 3;
      stopmoving = false;

      // motion smoothing, return to old motion and slow down, then go to new motion
      if(stopmoving == false && kinectVal < 5){
        kinectVal = Stop_old_movement();
        if(stopmoving == false && kinectVal < 5){
          kinectVal = Start_new_movement();
        }
      }*/

      break;
    
    case 'z':
      debug_pos = ST_LEFT;
      break;
      
    case 'c':
      debug_pos = ST_RIGHT;
      break;
      
     case 'f':
       debug_pos = wall_following();
       break;
       
     case 'y':
       debug_pos = -1;
       statusRequest = true;
       //returnAllData();
       break;
     
     case 'p':
       debug_pos = -1;
       if (DEBUG) {
         debugPrintln("Debug off");
         DEBUG = false;         
       } else {
         DEBUG = true;
         debugPrintln("Debug on");
       }
       break;
     
     case 48:  // number 0 (zero)
       //getSonarData();
       Wire.requestFrom(sonar_controller, 12);
  
       delay(100);
       debugPrint("Bytes available: ");
       debugPrintlnInt(Wire.available());
       
       if (Wire.available() == arraysize) {
         for( int i=0; i < arraysize; i++ ) {
           sonar[i] = Wire.read();
           debugPrintInt(sonar[i]);
           debugPrint(", ");
         }
         Serial.println("");
       } 
       
       break;

      // right now the only other case is an error so we return 0
    default:
      lrf_data[0] = 0;
      lrf_data[1] = 0;
      lrf_data[2] = 0;
      lrf_data[3] = 0;
      lrf_data[4] = 0;

      //Serial.write(lrf_data,5);
      debug_pos = -1;
      break;
  }
  
  return debug_pos;
}

void getSonarData() {
  //delay(200);
  boolean data_ok = false;
  
  while(!data_ok) {  // Keep requesting sonar data from UNO until no reading is 0
    Wire.requestFrom(sonar_controller, arraysize);
    
     delay(100);
     //debugPrint("Bytes available: ");
     //debugPrintlnInt(Wire.available());
     
     if (Wire.available() == arraysize) {
       for( int i=0; i < arraysize; i++ ) {
         sonar[i] = Wire.read();
         //if (sonar[i] == 0) {
         //  debugPrintln("reading sonar failed. Repeating ...");
         //  break;  // If reading 0, retry requesting from UNO
         //}
         //Serial.print(sonar[i]);
         debugPrintInt(sonar[i]);
         debugPrint(",");
         //Serial.print(", ");
         if (i==arraysize-1) data_ok = true;
       }
       //Serial.println("");
       debugPrintln("");
     } 
  }
}

int wall_following() {
  //Serial.println();
  boolean Fr_Obs = false;
  boolean Left_Obs = false;
  boolean Right_Obs = false;
  //DontMoveLeft = false;
  //DontMoveRight = false;
  //int right_dis, left_dis;
  int in_Left = 0, in_Right = 0, in_Front = 0, in_Back = 0;
  do {
    getSonarData();
    S1 = sonar[0]; //Front left sonar
    S3 = sonar[2]; //right sonar
    S4 = sonar[3]; //right sonar
    S9 = sonar[8]; //left sonar
    S10 = sonar[9];//left sonar
    S12 = sonar[11];//Front right sonar
    
    debugPrintInt(S1);
    debugPrint(" ");
    debugPrintInt(S3);
    debugPrint(" ");
    debugPrintInt(S4);
    debugPrint(" ");
    debugPrintInt(S9);
    debugPrint(" ");
    debugPrintInt(S10);
    debugPrint(" ");
    debugPrintlnInt(S12);
    
    delay(100);
    in_Left = digitalRead(bumper_left);
    in_Right = digitalRead(bumper_right);
    in_Front = digitalRead(bumper_front);
    in_Back = digitalRead(bumper_back);
    
    // IF no bumper is hit, then continue with wall following
    if (in_Left == HIGH && in_Right == HIGH && in_Front == HIGH && in_Back == HIGH) {
      digitalWrite(left_led,LOW);
      digitalWrite(right_led,LOW);
      digitalWrite(front_led,LOW);
      digitalWrite(back_led,LOW);
     
      debugPrint("\nRight distance: ");
      debugPrintlnInt(S10); //Print right distance
      debugPrint("Left distance: ");
      debugPrintlnInt(S3); // pring left distance
    
      
      if ((S1 < 10 && S1 > 0) || (S12 < 10 && S12 > 0)) Fr_Obs = true; else Fr_Obs = false;
      debugPrintln("checking front obstacle"); 
    
      if ((S3 < S10) || (S3 == S10)) {
        if (Fr_Obs) { 
          new_movement = ST_RIGHT;
          Left_Obs = true;
          Right_Obs = false;
          avoid(new_movement);
          debugPrintln("(Following Left) obstacle and moving right");
        //  goto start;
        } else { 
           if (Left_Obs){
             new_movement = FORWARD;
             Left_Obs = false;
             avoid(new_movement);
             debugPrintln("(Following Left) avoiding obstacle and moving forward");
            // goto start;
           } else { 
//No_ObsL:     
             if (S3 < 10 && S3 > 0) {
  
                // check bumper
                /*in = digitalRead(bumper_left);
                // Keep stopping while bumper is on
                while (in == LOW) {
                  goStop();
                  in = digitalRead(bumper_left);
                }*/
                //if (left_dis < 33) goto start;
                new_movement = ST_RIGHT;              
                //*getSonarData();
                //S1 = sonar[0];
                //*S3 = sonar[2];
                //S4 = sonar[3];
                //S9 = sonar[8];
                //*S10 = sonar[9];
                //S12 = sonar[11];         
                avoid(new_movement);
                debugPrintln("(Following Left) less than 10 and moving right");
                
                //goto No_ObsL;
             } else if (S3 >= 10) { 
               if (S3 > 20){
                                // Read from bumper
                  /*in = digitalRead(bumper_left);
                  // Keep stopping while bumper is on
                  while (in == LOW) {
                    goStop();
                    in = digitalRead(bumper_left);
                  }*/
                 new_movement = ST_LEFT;
                 //*getSonarData();
                 //S1 = sonar[0];
                 //*S3 = sonar[2];
                // S4 = sonar[3];
                // S9 = sonar[8];
                 //*S10 = sonar[9];
                // S12 = sonar[11];
                 avoid(new_movement);
                 debugPrintln("(Following Left) larger than 20 and moving left");
                  
                 //goto No_ObsL;
               } else {
                 new_movement = FORWARD;
                 debugPrintln("(Following Left) Forward"); 
                 avoid(new_movement);
             //  goto start;
               }
             }
           }
         }
      } else { 
        if (Fr_Obs){
          new_movement = ST_LEFT;
          Right_Obs = true;
          Left_Obs = false;
          debugPrintln("(Following right) obstacle and moving left");
          avoid(new_movement);
        //  goto start;
        } else { 
           if (Right_Obs) { 
             new_movement = FORWARD;
             Right_Obs = false;
             debugPrintln("(Following right) avoiding obstacle and moving forward");
             avoid(new_movement);
           //  goto start;
           } else {
//No_ObsR:     
             if (S10 < 10 && S10 > 0) {
                //if (right_dis < 33) goto start;
                new_movement = ST_LEFT;
                //*getSonarData();
               // S1 = sonar[0];
                //*S3 = sonar[2];
                //S4 = sonar[3];
                //S9 = sonar[8];
                //*S10 = sonar[9];
                //S12 = sonar[11];
                debugPrintln("(Following Right) less than 10 and moving left");
                avoid(new_movement);
                
                // Check bumper
                /*
                in = digitalRead(bumper_left);
                // Keep stopping while bumper is on
                while (in == LOW) {
                  goStop();
                  in = digitalRead(bumper_left);
                }*/
                //goto No_ObsR;
              } else if (S10 >= 10) { 
                    if (S10 > 20){
                       new_movement = ST_RIGHT;
                       //*getSonarData();
                       //S1 = sonar[0];
                       //*S3 = sonar[2];
                     //  S4 = sonar[3];
                      // S9 = sonar[8];
                       //*S10 = sonar[9];
                       //S12 = sonar[11];
                       debugPrintln("(Following Right) larger than 20 and moving right");
                       avoid(new_movement);
                       
                      // Read from bumper
                   /*   in = digitalRead(bumper_left);
                      // Keep stopping while bumper is on
                      while (in == LOW) {
                        goStop();
                        in = digitalRead(bumper_left);
                      }*/
                       
                       //goto No_ObsR;
                     }
                    else {
                       new_movement = FORWARD;
                       debugPrintln("(Following right) Forward"); 
                       avoid(new_movement);
                     //  goto start;
                     }
                   }
                 }
               }
        avoid(new_movement);
      } 
            
      debugPrint("Fr_Obs: ");
      debugPrintlnInt(Fr_Obs);
      debugPrint("Right_Obs: ");
      debugPrintlnInt(Right_Obs);
      debugPrint("Left_Obs: ");
      debugPrintlnInt(Left_Obs);          
    
    } else { 
        goStop(); // Stop the motors and indicate which bumpers has been tregered using the serial moniter and led's.
        if (in_Left == LOW)  {
          digitalWrite(left_led,HIGH);
          debugPrintln(":  LEFT BUMPER");
        }
        if (in_Right == LOW) {
          digitalWrite(right_led,HIGH);
          debugPrintln(":  RIGHT BUMPER");
        }
        if (in_Front == LOW) {
          digitalWrite(front_led,HIGH);
          debugPrintln(":  FRONT BUMPER");
        }
        if (in_Back == LOW)  {
          digitalWrite(back_led,HIGH);      
          debugPrintln(":  BACK BUMPER");
        }
    
     }
     
   } while(Serial.available() < 1);
   
   Serial.read();
   debugPrintln("Done wall following!");
   new_movement = STOP;
   avoid(new_movement);
   return -1;
 
}

void readInterrupt() {
    was_interrupt = digitalRead(interruptPin);    // Poll interrupt pin from sonar controller. 
    if(was_interrupt == true){
	//Serial.println("Interrupt true.");
        digitalWrite(interruptLED, HIGH);
        if(new_movement == FORWARD || new_movement == BACKWARD) {
             detected_obstacle();                         // jump to stop routine for detected obstacle
        }
        was_interrupt = false;
    }
    else {
      digitalWrite(interruptLED, LOW);
      //Serial.println("No interrupt.");
      /*if (avoid_count > 0) {
        //goForward(15);
        //delay(AVOID_DELAY);
        Serial.print("Avoid count: ");
        Serial.println(avoid_count);
        Serial.print("Direction: ");
        if(avoid_direction == 0) {
          Serial.println("RIGHT");
        } 
        else { Serial.println("LEFT");
        }
        returnToPath();
      }
      if (avoid_count == 0) {
        avoid_direction = 4;
      }*/
    }
}

/////////////////		Read Interrupt End		////////////////////////////

//////////////////////////// Begin LRF and Encoder subroutines ///////////

void lrfScan1()
{
   myservo.write(pos1);
   delay(1000);
   simpleread();
   //complicatedread();      
   
   myservo.write(pos2);
   delay(20);
   simpleread();
   //complicatedread();
   //delay(1000);
   myservo.write(center);
   delay(20);
   simpleread();
   //complicatedread();
   //delay(1000);         
   myservo.write(pos3);
   delay(20);
   simpleread();
   //complicatedread();
   //delay(1000);
   myservo.write(pos4);
   delay(20);
   simpleread();
   //complicatedread();
   //delay(1000);
   centerPos(); 
}

void lrfScan2()
{
  debugPrintln("\nLRF Scan2");
  // The ! is the arduino acknowledge for the LRF reading
  Serial.write('!');

  // Move servo, start LRF reading, store in outgoing array, repeat 5 times
  myservo.write(pos1);
  delay(120);
  Serial1.write('R');
  //lrf_data[0] = dataread();
  lrf_data[0] = complicatedread();
  lrf_data_buf[0] = lrf_data[0];
  //delay(1500);

  myservo.write(pos2);
  delay(120);
  Serial1.write('R');
  //lrf_data[1] = dataread();
  lrf_data[1] = complicatedread();
  lrf_data_buf[1] = lrf_data[1];
  //delay(1500);

  myservo.write(center);
  delay(120);
  Serial1.write('R');
  //lrf_data[2] = dataread();
  lrf_data[2] = complicatedread();
  lrf_data_buf[2] = lrf_data[2];
  //delay(1500);

  myservo.write(pos3);
  delay(120);
  Serial1.write('R');
  //lrf_data[3] = dataread(); 
  lrf_data[3] = complicatedread();
  lrf_data_buf[3] = lrf_data[3];
  //delay(1500);

  myservo.write(pos4);
  delay(120);
  Serial1.write('R');
  //lrf_data[4] = dataread();  
  lrf_data[4] = complicatedread();
  lrf_data_buf[4] = lrf_data[4];
  //delay(1500);
  
  
  // send out the data array
  centerPos();
  Serial.write(lrf_data,5);
  //delay(1000);
}

void readEncoder()
{
  // status byte for encoders
  uint8_t status;
  
  // bit to determine if encoder values are valid
  bool valid;
  
  // character buffer for opcode 
  //char pos;
  
  // local variables for caclulating odometry
  int32_t enc1;
  int32_t enc2;
  
  
  int32_t Dleft;
  int32_t Dright;
  
  // angle in radians for the trig functions
  float phi1fl;
  //Serial.println();
  //Serial.flush();
  while (Serial.available() > 0) {
    Serial.read();
  }
  // predetermined acknowledge char which motherboard requires to accept odometry data
  Serial.write('*');

  // read encoder, and set status and valid bits
  enc1= roboclaw.ReadEncM1(address, &status, &valid);
  if(valid) {

    // prevents underflow, since we are only looking for the relative change in enc cts and reseting it every loop,
    // if the enc subtracts from 0, it will end up overflowing the 16bit variable as well as being incorrect for our 
    // incremental case, for example an enc value reported as 4294967290 will be converted to -5 which is the true
    // relative change from 0 as we expect it.  This could possibly be improved by using the status byte which will
    // indicate an over or underflow.
    
    if (enc1 > (2^16)) {
      enc1 = enc1 - (2^32) + 1;
    }        

    // 1470cts/rev, 2.0944ft/rev, 3.2ft/map unit leads to 0.00044524 map units/encoder ct, or 44524 map units E-8/enc ct
    Dright = enc1*44524; 

    // debug prints
    /*Serial.print("Encoder1: ");
    Serial.print(enc1,DEC);
    Serial.print(", ");
    Serial.print(Dright,DEC);
    Serial.println(" ");*/
    debugPrint("Encoder1: ");
    debugPrintInt(enc1);
    debugPrint(", ");
    debugPrintlnInt(Dright);    
    
  }

  // read encoder 2, and set status and valid bits
  enc2= roboclaw.ReadEncM2(address, &status, &valid);      
  if(valid) {

    // prevents underflow, since we are only looking for the relative change in enc cts and reseting it every loop,
    // if the enc subtracts from 0, it will end up overflowing the 16bit variable as well as being incorrect for our 
    // incremental case, for example an enc value reported as 4294967290 will be converted to -5 which is the true
    // relative change from 0 as we expect it.
    if (enc2 > (2^16)) {
      enc2 = enc2 - (2^32) + 1;
    }

    // 1470cts/rev, 2.0944ft/rev, 3.2ft/map unit leads to 0.00044524 map units/encoder ct, or 44524 map units E-8/enc ct
    Dleft = enc2*44524;

    // debug prints
     /*debugPrint("Encoder2:");
     debugPrint((char *)enc2,DEC);
     debugPrint(", ");
     debugPrint((char *)Dleft,DEC);
     Serial.println(" ");*/
     debugPrint("Encoder2:");
     debugPrintInt(enc2);
     debugPrint(", ");
     debugPrintln((char *)Dleft);
     
  }

  // the following equations are taken from simreal.com/content/Odometry, and assume a two wheel 'tank-type' robot, which  
  // for MCECS bot means that all motion is assumed to have equal motion from the two left wheels and two right wheels. It
  // is certainly capable of other motion, but these equations MUST be rewritten to account for those motions.  They are 
  // derived by using the law of sines and assuming the distance is a straight line.

  // iterative calculation which accumulates the angular change of the robot from left and right encoder cts, where w is
  // the width of the two wheels
  phi1 = phi + (Dleft/w) - (Dright/w);

  // keeps angle wrapping around the circle either positively or negatively
  if (phi1 < 0) 
  {
    phi1 += 360;
  }
  else if (phi1 >= 360) 
  {
    phi1 -= 360;
  }

  // new change in distance, approximating curved path as a straight line.  The max speed of the robot and the sample period
  // for the encoders determines the amount of error, that is the faster the robot moves and the larger the period the larger 
  // the approximation error.  We determined that our max speed is sufficiently low and our sample period is sufficiently 
  // large such that the error is negligible.  If the speed or sample period are altered that may no longer be the case!!!
  D1 = (Dleft/2) + (Dright/2);

  // convert angle to radians and cast as float for trig functions
  phi1fl = float(phi1*PI/180);

  // iterative calculation which accumulates the x, y position of the robot based on these previous values obtained from 
  // encoder cts 
  x1 = x + D1*sin(phi1fl);
  y1 = y + D1*cos(phi1fl);

  // iterate the transform variables
  x=x1;
  y=y1;
  phi=phi1;      

  x1 /= 1000000;
  y1 /= 1000000;

  // debug prints
   debugPrint("Distance: ");
   //Serial.println(D1);
   debugPrintlnInt(D1);
   debugPrint("X coordinate: ");
   //Serial.println(x1,DEC);
   debugPrintlnInt(x1);
   debugPrint("Y coordinate: ");
   //Serial.println(y1,DEC);
   debugPrintlnInt(y1);
   debugPrint("Angle in radians: ");
   
   if (DEBUG) {  // Float number - can't use debugPrint* - function doesn't accept float
     Serial.print(phi1fl);
   }
   debugPrint("Angle in degrees: ");
   //Serial.println(phi1,DEC);
   debugPrintlnInt(phi1);
   

  byte package[6];
  package[0] = x1 & 255;
  if (x1 < 0) { 
    package[1] = ((x1 >> 8) & 255) | 128;
  }
  else
  {
    package[1] = ((x1 >> 8) & 255) & 127;
  }
  package[2] = y1 & 255;
  if (y1 < 0) 
  {
    package[3] = ((y1 >> 8) & 255) | 128; 
  }
  else
  { 
    package[3] = ((y1 >> 8) & 255) & 127;
  }  
  package[4] = phi1 & 255;
  package[5] = (phi1 >> 8) & 255;      

  Serial.write(package,6);
  //Serial.flush();
  while (Serial.available() > 0) {
    Serial.read();
  }

  // reset the encoders so that we are recieving incremental change in encoder cts
  roboclaw.ResetEncoders(address);
  delay(1000);
}

void centerPos()
{
  myservo.write(center);
}

// A simple method to read from the LRF
void simpleread() {
  debugPrintln("Reading LRF ...");
        char readlrf;
        char readbuffer[4];
        long result;
        boolean gotresult = false;
        // Sometimes serial buffer is not empty - clear it out first or you'll get erroneous data
        debugPrintln("Flushing ... ");
        while(Serial1.available() > 0) {
          Serial1.read();
          debugPrint(".");
        }
        debugPrint("done.\n");
          
        delay(10);
        Serial1.write('R');
        int count = 0;
        boolean lt15 = false;
        while(Serial1.available() < 15) {
          if (!lt15) {
            debugPrintln("Serial1.available < 15 ...");          
            lt15 = true;
          }
          //delay(100);
          count = count + 1;
          if (count > 20) {
            //delay(150);
            Serial1.write('R');
            delay(50);
            count = 0;
          }
        }
        //delay(1000);
        debugPrint("Serial1.available() = ");
        debugPrintlnInt(Serial1.available());
        while (Serial1.available() > 0) {
          
          //delay(150);
          readlrf = Serial1.read();
          delay(10);
          if (readlrf == ':') {
            //debugPrintln(":");
            readlrf = Serial1.read();
          }          
          
          debugPrint((char*)readlrf);
        }
        
        //Serial1.flush();
        debugPrintln("Done!");
}

// A more complicated method to read from LRF, the output is formatted in long type
long complicatedread() {
  //debugPrintln("Reading LRF ...");
  char readlrf;
  char readbuffer[4];
  long result;
  boolean gotresult = false;
  //Serial.println();
  // Sometimes serial buffer is not empty - clear it out first or you'll get erroneous data
  //debugPrintln("Flushing ... ");
  while(Serial1.available() > 0) {
    Serial1.read();
    //debugPrint(".");
  }
  //debugPrintln("done.");
    
  delay(10);
  Serial1.write('R');
  int count = 0;
  //debugPrintln("Scanning ... \n");
  while(Serial1.available() < 15) {
    //Serial.println("Serial1.available < 15");          
    //Serial.print(".");
    delay(10);
    
    // keep reading until Serial1.available() >= 15
    count = count + 1;
    // if already checked 20 times (arbitrary number) and still failing, send the command again
    if (count > 50) {
      //delay(150);
      Serial1.write('R');
      delay(20);
      count = 0;
    }
  }
  if (Serial1.available() == 16) {
    //debugPrintln("LRF scan failed.\n");
    return 0;
  }
  //Serial.write("\n");
  //delay(1000);
  //Serial.write("Serial1.available() = ");
  //Serial.println(Serial1.available());
  while (Serial1.available() > 0) {
    
    //delay(150);
    readlrf = Serial1.read();
    delay(10);
    if (readlrf == ':') {
      //Serial.println(':');
      readlrf = Serial1.read();
    }
    //Serial.print("readlrf: ");
    //Serial.println((char)readlrf);
    
    // Format of response from lrf: "D = 0000 mm"
    if ((readlrf == 'D') && !gotresult) {
      Serial1.read();  // space
      Serial1.read();  // '='
      Serial1.read();  // space
      readbuffer[0] = Serial1.read();
      readbuffer[1] = Serial1.read();
      readbuffer[2] = Serial1.read();
      readbuffer[3] = Serial1.read();
      // in mm: result = (readbuffer[0]-'0')*1000 + (readbuffer[1]-'0')*100 + (readbuffer[2]-'0')*10 + (readbuffer[3]-'0');
      result = (readbuffer[0]-'0')*100 + (readbuffer[1]-'0')*10 + (readbuffer[2]-'0');
      debugPrint("Distance: ");
      debugPrintInt(result);
      debugPrint(" cm\n");
      gotresult = true;
      break;
    //delay(200);
    }
    
    //Serial.print(readlrf);
  }
  //Serial.write("Flushing ... ");
  while(Serial1.available()) {
    Serial1.read();
  }
  //Serial1.flush();
  //Serial.println("Done!");
  return result;
}

// dataread will read the bytes from the LRF and buffer them, then assemble them into a value in cm
long dataread() {
  long result;
  char range[15];
  //Serial.println("datareading...");
  debugPrintln("reading from LRF...");  
  while (Serial1.available() < 15) {
    debugPrint(".");
  }
  for (int i = 0; i < 15; i++) {
    range[i] = Serial1.read();
    delay(50);
    
  }
  

  //round the mm value to an integer cm value
  if (range[10] < 6) {
    result = (range[7]-'0')*100 + (range[8]-'0')*10 + (range[9]-'0');
  }
  else
  {
    result = (range[7]-'0')*100 + (range[8]-'0')*10 + (range[9]-'0') + 1;
  }

  // if the measurement is outside the valid range of the LRF return a 0
  if (result > 240 || result < 15) {
    result = 0;
  }

  // flush buffer
  for (int j = 0; j < 15; j++) {
    range[j] = 0;
  }

  return result;
}
/////////////////		POST start		////////////////////////////

// This code turns the robot right then left to show the robot is in working order and able to move

void POST() {
	debugPrint("POST Test Start");
	rotateRight(10);
	delay(500);
	rotateLeft(10);
	delay(500);
	goStop();
	delay(500);
	debugPrint("POST Test End");	
}

/////////////////		POST end		///////////////////////////////

/////////////////		Do Move start		////////////////////////////////

void doMove() {
	if(new_movement == STOP && old_movement != STOP) {
        //debugPrintln("new_movement==STOP && old_movement!=STOP");
		while(MotorSpeed > 10) {
                  MotorSpeed = MotorSpeed - increment;
                  function[old_movement](MotorSpeed);
                  delay(1);
                }
                MotorSpeed = 0;
		goStop();
                //old_movement = new_movement;
                //return true;
	}
	else if(new_movement != old_movement) {
        //debugPrintln("new_movement!=old_movement");
                while(MotorSpeed > 10) {
                  MotorSpeed = MotorSpeed - increment;
                  function[old_movement](MotorSpeed);
                  delay(1);
                }
		function[new_movement](MotorSpeed);
                //old_movement = new_movement;
                //return true;
	}
	else if(MotorSpeed < TopMotorSpeed && old_movement != STOP) {
        //debugPrintln("MotorSpeed<TopMotorSpeed && old_movement!=STOP");
	  MotorSpeed = MotorSpeed + increment;
          if(MotorSpeed > TopMotorSpeed){
            MotorSpeed = TopMotorSpeed;
            //return true;
          }            
	  function[new_movement](MotorSpeed);
          //old_movement = new_movement;
          //return false;
	}
        
        // DEBUG OUTPUT
        /*Serial.print("Old_movement= ");
        Serial.print(old_movement);
        Serial.print(", new_movement= ");
        Serial.println(new_movement);*/
        /*debugPrint("Old_movement= ");
        debugPrint((char*)old_movement);
        debugPrint(", new_movement= ");
        debugPrintlnInt(new_movement);*/
        
        old_movement = new_movement;
}

/////////////////	Do Move End		////////////////////////////////

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
              debugPrintln("stopping old movement ...");
              MotorSpeed = MotorSpeed - increment;
              first_iteration = false;
              function[old_movement](MotorSpeed);

              if (MotorSpeed == NoMotorSpeed && stopmoving == true){
                new_movement = old_movement;
                debugPrintln("updated kinect value from STOP");
              }
               else if (MotorSpeed == NoMotorSpeed){
              old_movement = new_movement;
              debugPrintln("updated kinect value from STOP");
              }

              if(Serial.available()){
                 update = Serial.read();
                 if (update != new_movement && update != 10){
                 new_movement = update;
                 //Serial.println(update);
                 //Serial.println("Interrupted Stop Movement"); 
                 debugPrintlnInt(update);
                 debugPrintln("Interrupted Stop Movement");
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
					debugPrintln("updated kinect from START");
				}

               if(Serial.available()){
                 update = Serial.read();
                 if (update != new_movement && update != 10){
                 old_movement = new_movement;
                 new_movement = update;
                 //Serial.println(update);
                 //Serial.println("Interrupted Start Movement"); 
                 debugPrintlnInt(update);
                 debugPrintln("Interrupted Start Movement");
                 break;
                 } 
                }
              }
       
  return new_movement;
}

// routine to slow down old motion for motor commands
/*int Stop_old_movement(){ 
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
}*/

// routine to smoothly start new motion
/*int Start_new_movement(){
  update = new_movement;
  //Serial.print("pressed: ");
  //Serial.println(new_movement);
  for( MotorSpeed; MotorSpeed <= TopMotorSpeed; MotorSpeed = MotorSpeed + increment){

    function[new_movement](MotorSpeed);

    if(MotorSpeed == TopMotorSpeed){
      old_movement = new_movement; 
      //Serial.println("updated kinect from START");
    }

    if(Serial.available()){
      update = Serial.read();
      if (update != new_movement && update != 10){
        old_movement = new_movement;
        new_movement = update;
        //Serial.println("Interrupted Start Movement"); 
        break;
      } 
    }
  }
  return new_movement;
}*/

void rotateRight(uint8_t MotorSpeed){
  debugPrint("Rotate Right: ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.BackwardM1(0x80, MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
}

void rotateLeft(uint8_t MotorSpeed){
  debugPrint("Rotate Left:   ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.ForwardM1(0x80, MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
}


void strafeLeft(uint8_t MotorSpeed){
  debugPrint("Strafe Left:   ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.ForwardM1(0x80, MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
}


void strafeRight(uint8_t MotorSpeed){
  debugPrint("Strafe Right:   ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.BackwardM1(0x80,MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
}


void goForward(uint8_t MotorSpeed){
  debugPrint("Forward:   ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.ForwardM1(0x80,MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
  Wire.beginTransmission(sonar_controller);
  Wire.write(new_movement);
  Wire.endTransmission();
}

void goBackward(uint8_t MotorSpeed){
  debugPrint("Backward:   ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.BackwardM1(0x80,MotorSpeed);
  roboclaw.BackwardM2(0x80,MotorSpeed);
  roboclaw.BackwardM1(0x81,MotorSpeed);
  roboclaw.BackwardM2(0x81,MotorSpeed);
  Wire.beginTransmission(sonar_controller);
  Wire.write(new_movement);
  Wire.endTransmission();
}

void goForwardDiag(uint8_t MotorSpeedRight, uint8_t MotorSpeedLeft){
  debugPrint("Diagonal Forward:   ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.ForwardM1(0x80,MotorSpeedRight);
  roboclaw.ForwardM2(0x80,MotorSpeedLeft);
  roboclaw.ForwardM1(0x81,MotorSpeedRight);
  roboclaw.ForwardM2(0x81,MotorSpeedLeft);
}

void goBackwardDiag(uint8_t MotorSpeedRight, uint8_t MotorSpeedLeft){
  debugPrint("Diagonal Reverse:   ");
  debugPrintlnInt(MotorSpeed);
  roboclaw.BackwardM1(0x80,MotorSpeedRight);
  roboclaw.BackwardM2(0x80,MotorSpeedLeft);
  roboclaw.BackwardM1(0x81,MotorSpeedRight);
  roboclaw.BackwardM2(0x81,MotorSpeedLeft);
}

void goStop(){
  debugPrint("Stopping");
  MotorSpeed = 0;
  roboclaw.ForwardM1(0x80,MotorSpeed);
  roboclaw.ForwardM2(0x80,MotorSpeed);
  roboclaw.ForwardM1(0x81,MotorSpeed);
  roboclaw.ForwardM2(0x81,MotorSpeed);
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
  //uint8_t sonar_number, obstacle;
  int interrupted_movement;
  interrupted_movement = new_movement;

  debugPrint("Obstacle Detected:   ");
  
  if(old_movement == BACKWARD) {
    //new_movement = FORWARD;
    avoid(FORWARD);
  }
  else {
    //new_movement = BACKWARD;
    avoid(BACKWARD);
  }
  
  avoid(STOP);
   
  //char sonar_number, obstacle;
  Wire.requestFrom(sonar_controller, 2);
  //int i = 0;
  delay(100);
  debugPrint("Bytes available: ");
  debugPrintlnInt(Wire.available());
  
   delay(100);
   obstacle = Wire.read();
   delay(100);
   sonar_number = Wire.read();
   debugPrint("\nobstacle: ");
   debugPrintInt(obstacle);
   debugPrint(" sonar number:  "); 
   debugPrintlnInt(sonar_number);
   
   ////////  evasive manuever method /////
   decide(obstacle, sonar_number);
   
   ////////  End of evasive manuever method /////

	 new_movement = interrupted_movement;
   debugPrintln("end detected_obstacle()");
}

// End Detected Obstacle Function // 

// Decide what to do / where to move when there's an obstacle
void decide(int obstacle, int sonar_number) {
  // Sonars:
  // front left: 1,2,3
  // back left:  4,5,6
  // back right: 7,8,9
  // front right: 10,11,12
  debugPrint("Obstacle: ");
  debugPrintlnInt(obstacle);
  debugPrint("Sonar number: ");
  debugPrintlnInt(sonar_number);
  debugPrint("Trying to decide what to do...");
  if ((sonar_number == 12) || (sonar_number == 11)) {
    debugPrintln("Go Left");
    avoid(LEFT);
  } else if ((sonar_number == 7) || (sonar_number == 6) || (sonar_number == 5)) {
    debugPrintln("Go Right");
    avoid(RIGHT);
  } else if ((sonar_number == 2) || (sonar_number == 3) || (sonar_number == 4)) {
    debugPrintln("Go Left");
    avoid(LEFT);
  } else if ((sonar_number == 8) || (sonar_number == 9) || (sonar_number == 10)) {
    debugPrintln("Go Right");
    avoid(RIGHT);
  } else {
    avoid(STOP);
  }
}

void avoid(int direction) {
  long now = millis();
  int maxtime = 801;
  int mintime = 400;
  new_movement = direction;
  while (millis() - now < random(mintime,maxtime)) {
    doMove();
  }
}

// Mathias' code to read arduino data from Processing/Kinect program
void returnAllData() {
  byte interrupted = 0;
  if (was_interrupt) {
    interrupted = 1;
  } else {
    interrupted = 0;
  }
  byte stoppedmoving = 0;
  if (stopmoving) {
    stoppedmoving = 1;
  } else {
    stoppedmoving = 0;
  }
  getSonarData();
  // byte 33 = '!'
  byte dataArray[22] = { 35, interrupted, stoppedmoving, (byte)new_movement, 
                        lrf_data_buf[0], lrf_data_buf[1], lrf_data_buf[2], 
                        lrf_data_buf[3], lrf_data_buf[4], 
                        (byte)sonar[0], (byte)sonar[1],(byte)sonar[2],(byte)sonar[3],
                        (byte)sonar[4],(byte)sonar[5],(byte)sonar[6],(byte)sonar[7],
                        (byte)sonar[8],(byte)sonar[9],(byte)sonar[10],(byte)sonar[11], 36}; // need to add orientation and all sonar data
                        
  Serial.write(dataArray, 22);
  /*Serial.println((char)33);
  Serial.println(interrupted);
  Serial.println(stoppedmoving);
  Serial.println(new_movement);
  Serial.println(lrf_data_buf[0]);
  Serial.println(lrf_data_buf[1]);
  Serial.println(lrf_data_buf[2]);
  Serial.println(lrf_data_buf[3]);
  Serial.println(lrf_data_buf[4]);
  Serial.println(obstacle);
  Serial.println(sonar_number);
  Serial.println((char)37);*/
}

void debugPrint(char* toPrint) {
  if(DEBUG) {
    Serial.print(toPrint);
  }
}

void debugPrintln(char* toPrint) {
  if(DEBUG) {
    Serial.println(toPrint);
  }
}

void debugPrintInt(int toPrint) {
  if(DEBUG) {
    Serial.print(toPrint);
  }
}

void debugPrintlnInt(int toPrint) {
  if(DEBUG) {
    Serial.println(toPrint);
  }
}
