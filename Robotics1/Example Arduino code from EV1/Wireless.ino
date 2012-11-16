// Jesse Adams 
// ECE 510 
// 10-21-2012
// Graduate Project

/**************   Remote Control Car Code ***************/
/* This code is for general control of an RC car        */


#include <AFMotor.h>  // Library provided by Adafruit for "motor shield" 
#include <Servo.h>    // Standard Servo Library with Arduino
#include <SoftwareSerial.h> // Built-in Library for serial transmissions


/******** Initializing the variables **********/ 
SoftwareSerial mySerial = SoftwareSerial (19, 18); // Transmission pins
Servo myservo;                       // Create Servo Motor 
AF_DCMotor motor2(2, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor motor1(1, MOTOR12_64KHZ); // create motor #1, 64KHz pwm
int Photo_tran = 14;                 // Phototransistor for RPM
int Emitter = 15;                    // LED for Phototransistor

/**** Control Variables ****/ 
  int incomingbyte, scan;
  int Fast = 4400;
  int Medium = 3000;
  int Slow = 2000; 
  int Stop = 0; 
  int Left = 1100; 
  int Straight = 1500; 
  int Right = 1800;
  int desiredRPM;  
  int Turn, count; 

/******** Initializing Arduino Pins **********/
void setup() {
  Serial.begin(57600);        // set up Serial library at 9600 bps
  myservo.attach(9);          // set servo motor to pin 9 
  myservo.write(90);          // set servo to mid-point
  pinMode(Emitter, OUTPUT);   // set the LED pin to output
  digitalWrite(Emitter, HIGH);// set the LED on 
  pinMode(Photo_tran, INPUT); // set the photo transistor pin to input
  mySerial.begin(9600);       // set Xbee Baud rate
}

/******** Program **********/
void loop() {

  scan++;
  
  if (10000 == scan) {
    mySerial.print("incomingbyte: ");
    mySerial.print(incomingbyte); mySerial.print("\r");
    scan = 0;
  }
  if (mySerial.available()) {
    incomingbyte = mySerial.read();
  }
  
    switch (incomingbyte) {
      case 107:
        myservo.write(90);    // Wheels Forward. User pushed key "k"
        break;
      
      case 106:
        myservo.write(50);  // Wheels Left. User pushed key "j"
        break;
      
      case 108:
        myservo.write(120);  // Wheels Right. User pushed key "l"
        break;
      
      case 105:
    motor2.setSpeed(175);       // set the motor speed
    motor1.setSpeed(175);       // set the motor speed
    motor1.run(FORWARD);        // User pushed key "i"
    motor2.run(FORWARD);        // Turn on motors going Forward
        break;  
        
      case 44:
    motor2.setSpeed(175);       // set the motor speed
    motor1.setSpeed(175);       // set the motor speed
    motor1.run(BACKWARD);       // User pushed key ","
    motor2.run(BACKWARD);       // Turn on motors going backward
        break;  
        
      case 32:
    motor2.setSpeed(0);         // set the motor speed
    motor1.setSpeed(0);         // set the motor speed
    motor1.run(RELEASE);        // User pushed "Space Bar"
    motor2.run(RELEASE);        // Stop the motors
        break;    
         default:
           break;
    }

}
