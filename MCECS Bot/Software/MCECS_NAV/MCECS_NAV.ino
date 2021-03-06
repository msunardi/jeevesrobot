
/* This program accepts a command from a host program using MRPT, either an 'l' or an 'e', and returns a 5pt LRF scan or odometry (x,y,phi) 
   March 2013 by Mitch Barton
   V1.0
 */

#include <Servo.h> 
// #include <SoftwareSerial.h>  don't include because we are using BMserial and there's a conflict
#include <BMSerial.h>
#include <RoboClaw.h>

// opcode which determines case to execute
char pos;

// outgoing final LRF range measurements
byte data[5];                    

// positions for this particular servo with this particular mounting 
// based on center
int center = 100;              
int pos1 = center + 20;
int pos2 = center + 10;
int pos3 = center - 10;
int pos4 = center - 20;

// address for motor controller
#define address 0x80

// constants for PID on motor controllers
#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define qpps 44000

// variables for transformation from encoders to odometry (x, y, phi)
int32_t phi = 0;
int32_t phi1 = 0;
int32_t D1 = 0;
int32_t x1 = 0;
int32_t y1 = 0;
int32_t x = 0;
int32_t y = 0;

// the width of the wheels is 17" which converts to 0.4427 map units, 
// but we've also added the scale factor for converting to map units E-8, 
// as well as converting from radians to degrees
const int32_t w = 44270000*PI/180;

// assign Rx Tx serial comm for the motor controllers to pins 5 and 6
RoboClaw roboclaw(50,52);

// Setup RX TX lines for LRF

// create servo object
Servo myservo;                 

void setup()
{
  
  // open PC/arduino serial port at 9600 baud
  Serial.begin(9600);

  // open the software serial port to the LRF at 9600 baud
  Serial1.begin(9600);       
  
  // open the software serial port for the roboclaw
  roboclaw.begin(2400);    
  
  // attaches the servo on pin 9 to the servo object
  myservo.attach(9);       
  
  // initialize the LRF
  delay(2000);
  Serial1.write('U');
  delay(100);
  Serial1.read();
  
  // set PID constants for motor controller 
  roboclaw.SetM1Constants(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(address,Kd,Kp,Ki,qpps);
  
}

void loop()
{
  
  // status byte for encoders
  uint8_t status;

  // bit to determine if encoder values are valid
  bool valid;

  // character buffer for opcode 
  char pos;

  // local variables for caclulating odometry
  int32_t enc1;
  int32_t enc2;
  int32_t Dleft;
  int32_t Dright;

  // angle in radians for the trig functions
  float phi1fl;
  
  // if we get a valid byte, write it to our local variable  
  if (Serial.available() > 0) {    
    delay(150);
    pos = Serial.read();           

    // depending what that byte is, perform an action     
    switch (pos) {

      // l is the agreed character to start the standard 5pt LRF reading 
    case 'l':
      
      // The ! is the arduino acknowledge for the LRF reading
      Serial.write('!');
      
      // Move servo, start LRF reading, store in outgoing array, repeat 5 times
      myservo.write(pos1);
      delay(150);
      Serial1.write('R');
      data[0] = dataread();
      delay(1500);
 
      myservo.write(pos2);
      delay(150);
      Serial1.write('R');
      data[1] = dataread();    
      delay(1500);
      
      myservo.write(center);
      delay(150);
      Serial1.write('R');
      data[2] = dataread();
      delay(1500);
      
      myservo.write(pos3);
      delay(150);
      Serial1.write('R');
      data[3] = dataread(); 
      delay(1500);
      
      myservo.write(pos4);
      delay(150);
      Serial1.write('R');
      data[4] = dataread();  
      delay(1500);
      
      // send out the data array
      Serial.write(data,5);

      break;
      
    case 'e':

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

        /* debug prints
        Serial.print("Encoder1:");
        Serial.print(enc1,DEC);
        Serial.print(" ");
        Serial.print(Dright,DEC);
        Serial.println(" ");
        */
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

        /* debug prints
        Serial.print("Encoder2:");
        Serial.print(enc2,DEC);
        Serial.print(" ");
        Serial.print(Dleft,DEC);
        Serial.println(" ");
        */
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
      x1 = x + D1*cos(phi1fl);
      y1 = y + D1*sin(phi1fl);

      // iterate the transform variables
      x=x1;
      y=y1;
      phi=phi1;      

      x1 /= 1000000;
      y1 /= 1000000;

      /* debug prints
      Serial.print("Distance: ");
      Serial.println(D1);
      Serial.print("X coordinate: ");
      Serial.println(x1,DEC);
      Serial.print("Y coordinate: ");
      Serial.println(y1,DEC);
      Serial.print("Angle in radians: ");
      Serial.print(phi1fl);
      Serial.print("Angle in degrees: ");
      Serial.println(phi1,DEC);
      */

        
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
       
      Serial.write(package,7);
      

      // reset the encoders so that we are recieving incremental change in encoder cts
      roboclaw.ResetEncoders(address);

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
  char range[15];  
  while (Serial1.available() < 15) {
  }
  for (int i = 0; i < 15; i++) {
    range[i] = Serial1.read();
    delay(50);
  }
  Serial1.flush();
  
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


