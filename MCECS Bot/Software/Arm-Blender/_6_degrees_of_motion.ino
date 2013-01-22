/*
Arduino Servo Test sketch
 */
#include <Servo.h>

// Declare 6 Servos, one for each degree of freedom
Servo servoLiftWrist; 
Servo servoTwistElbow;
Servo servoLiftElbow;
Servo servoTwistShoulder;
Servo servoLiftShoulder;
Servo servoSwingShoulder;

// variable to determine if the whole sting has been recieved
int numAnglesReceived = 0;
int numberOfServos = 6;
boolean stringComplete = false;

// Strings recieved from Blender
String LiftWristString = "";
String LiftElbowString = "";
String TwistElbowString = "";
String TwistShoulderString = "";
String LiftShoulderString = "";
String SwingShoulderString = "";

// Variable to hold how many bytes were recieved for each servo
int LiftWristBytes = 0;
int LiftElbowBytes = 0;
int TwistElbowBytes = 0;
int TwistShoulderBytes = 0;
int LiftShoulderBytes = 0;
int SwingShoulderBytes = 0;

// int variables to convert above strings to ints to send to servos
int LiftWristAngle = 0;
int LiftElbowAngle = 0;
int TwistElbowAngle = 0;
int TwistShoulderAngle = 0;
int LiftShoulderAngle = 0;
int SwingShoulderAngle = 0; 

void setup()
{
  // Attach the servos to the pwm pins of the Arduino
  servoLiftWrist.attach(3); 
  servoTwistElbow.attach(5); 
  servoLiftElbow.attach(6);
  servoTwistShoulder.attach(9);
  servoLiftShoulder.attach(10);
  servoSwingShoulder.attach(11);

  //initialize the serial port
  Serial.begin(115200);
  Serial.flush();
  
  // Reserve memory for the incomming angles
  LiftWristString.reserve(10);
  LiftElbowString.reserve(10);
  TwistElbowString.reserve(10);
  TwistShoulderString.reserve(10);
  LiftShoulderString.reserve(10);
  SwingShoulderString.reserve(10);
  
  // Put servos in the default positions
  servoLiftWrist.write(90); 
  servoTwistElbow.write(90);
  servoLiftElbow.write(0);
  servoTwistShoulder.write(90);
  servoLiftShoulder.write(170);
  servoSwingShoulder.write(10);
  
}

int angle = 0;
int angle2;
int num;
int lengthofString = 0;
int byteCount = 0;
void loop()
{
  // wait for the servo angle
  if (stringComplete)
  {
    // convert the strings to ints, this should be a function
    // but I found that the int(str) function didn't work properly
    for (int i = 0; i < (LiftWristBytes-1); i++) 
        LiftWristAngle = LiftWristAngle * 10 + (LiftWristString[i] - '0');
    for (int i = 0; i < (TwistElbowBytes-1); i++) 
        TwistElbowAngle = TwistElbowAngle * 10 + (TwistElbowString[i] - '0');
    for (int i = 0; i < (LiftElbowBytes-1); i++) 
        LiftElbowAngle = LiftElbowAngle * 10 + (LiftElbowString[i] - '0');
    for (int i = 0; i < (TwistShoulderBytes-1); i++) 
        TwistShoulderAngle = TwistShoulderAngle * 10 + (TwistShoulderString[i] - '0');
    for (int i = 0; i < (LiftShoulderBytes-1); i++) 
        LiftShoulderAngle = LiftShoulderAngle * 10 + (LiftShoulderString[i] - '0');
    for (int i = 0; i < (SwingShoulderBytes-1); i++) 
        SwingShoulderAngle = SwingShoulderAngle * 10 + (SwingShoulderString[i] - '0');
        
    // Write angles to the servos
    servoLiftWrist.write(LiftWristAngle); 
    servoTwistElbow.write(TwistElbowAngle);
    servoLiftElbow.write(LiftElbowAngle);
    servoTwistShoulder.write(TwistShoulderAngle);
    servoLiftShoulder.write(LiftShoulderAngle);
    servoSwingShoulder.write(SwingShoulderAngle);
    
    // Clear Angles and Strings
    LiftWristAngle = 0;
    LiftElbowAngle = 0;
    TwistElbowAngle = 0;
    TwistShoulderAngle = 0;
    LiftShoulderAngle = 0;
    SwingShoulderAngle = 0;
    

    LiftWristString = "";
    LiftElbowString = "";
    TwistElbowString = "";
    TwistShoulderString = "";
    LiftShoulderString = "";
    SwingShoulderString = "";
    
    LiftWristBytes = 0;
    LiftElbowBytes = 0;
    TwistElbowBytes = 0;
    TwistShoulderBytes = 0;
    LiftShoulderBytes = 0;
    SwingShoulderBytes = 0;
    
    stringComplete = false;
    numAnglesReceived =0;
  }
}

void serialEvent(){
 // Serial.println("GOT INTO SERIAL EVENT");
 // when a serial message is waiting
  while (Serial.available()){
    // count each angle till we have all six.
    //  newline character (/n) denotes seperation of angles 
    if(numAnglesReceived <  (numberOfServos+1)){
      
      // get the serial string sent from Blender one char at a time 
      char inChar = (char)Serial.read();
      Serial.println(inChar);
    
      // ignore the character if it is b or ' else decode
      if (inChar != 'b' and inChar != '\''){
        // If it is the first angle 
        if(numAnglesReceived == 1){
          LiftWristString += inChar;
          LiftWristBytes += 1;}
          
        else if (numAnglesReceived == 2){
          TwistElbowString += inChar;
          TwistElbowBytes += 1;}
          
        else if (numAnglesReceived == 3){
          LiftElbowString += inChar;
          LiftElbowBytes += 1;}
        
        else if (numAnglesReceived == 4){
          TwistShoulderString += inChar;
          TwistShoulderBytes += 1;}
        
        else if (numAnglesReceived == 5){
          LiftShoulderString += inChar;
          LiftShoulderBytes += 1;}
          
        else if (numAnglesReceived == 6){
          SwingShoulderString += inChar;
          SwingShoulderBytes += 1;}
          
        if(inChar == '\n'){
            numAnglesReceived ++;
            Serial.println(numAnglesReceived);
          }
        if(numAnglesReceived == 7){
          stringComplete = true;
        }
  }
}
}
}

