/* This program accepts a command (L, C, or R) on an arduino serial port and commands a servo to move to the desired location, and take a range measurement with the LRF.
 */

#include <Servo.h> 
#include <SoftwareSerial.h>

SoftwareSerial myLRF (10, 11); // Setup RX TX lines for LRF
Servo myservo;                 // create servo object to control a servo 
// a maximum of eight servo objects can be created 
char pos;
byte data[5];                    // incoming range measurement
int center = 100;              // position for this particular servo with this particular mounting to be pointed center
int pos1 = center + 20;
int pos2 = center + 10;
int pos3 = center - 10;
int pos4 = center - 20;

void setup()
{

  Serial.begin(9600);      // start PC/arduino serial port at 9600 baud
  myLRF.begin(9600);       // start the software serial port to the LRF at 9600 baud
  myservo.attach(9);       // attaches the servo on pin 9 to the servo object
  // initialize the LRF
  delay(2000);
  myLRF.write('U');
  delay(100);
  myLRF.flush();
  
}

void loop()
{
  // if we get a valid byte, write it to our local variable  
  if (Serial.available() > 0) {    
    delay(150);
    pos = Serial.read();           

    // depending what that byte is, perform an action     
    switch (pos) {

      // l is the agreed character to start the standard 5pt LRF reading 
    case 'l':
      Serial.write('!');
      
      myservo.write(pos1);
      delay(150);
      myLRF.write('R');
      data[0] = dataread();
 
      myservo.write(pos2);
      delay(150);
      myLRF.write('R');
      data[1] = dataread();    
      
      myservo.write(center);
      delay(150);
      myLRF.write('R');
      data[2] = dataread();
      
      myservo.write(pos3);
      delay(150);
      myLRF.write('R');
      data[3] = dataread(); 
      
      myservo.write(pos4);
      delay(150);
      myLRF.write('R');
      data[4] = dataread();  
   
      Serial.write(data,5);

      break;
    
    // right now the only other case is an error so we return 0
    default:
      data[0] = 0;
      data[1] = 0;
      data[2] = 0;
      data[3] = 0;
      data[4] = 0;

      Serial.write(data,5);      
      
    }
  }
}  

// dataread will read the bytes from the LRF and buffer them, then assemble them into a value in cm
long dataread() {
  long result;
  char range[12];  
  myLRF.flush();
  while (myLRF.available() < 12) {
  }
  for (int i = 0; i < 12; i++) {
    range[i] = myLRF.read();
    delay(50);
  }
  myLRF.flush();
  
  //round the mm value to an integer cm value
  if (range[8] < 6) {
    result = (range[5]-'0')*100 + (range[6]-'0')*10 + (range[7]-'0');
  }
  else
  {
    result = (range[5]-'0')*100 + (range[6]-'0')*10 + (range[7]-'0') + 1;
  }
  
  // if the measurement is outside the valid range of the LRF return a 0
  if (result > 240 || result < 15) {
    result = 0;
  }
  
  // flush buffer
  for (int j = 0; j < 12; j++) {
    range[j] = 0;
  }

  return result;
}


