#include <BMSerial.h>
#include <RoboClaw.h>

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
const uint32_t w = 44270000*PI/180;

// assign Rx Tx serial comm for the motor controllers to pins 5 and 6, respectively
RoboClaw roboclaw(5,6);

void setup() {
  // open serial comms with control PC and motor controllers
  Serial.begin(9600);
  roboclaw.begin(2400);

  // set PID constants for motor controller 
  roboclaw.SetM1Constants(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2Constants(address,Kd,Kp,Ki,qpps);
}

void loop() {
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

  // angle in radians
  float phi1fl;

  if (Serial.available() > 0) {    
    delay(150);
    pos = Serial.read();           

    // depending what that byte is, perform an action     
    switch (pos) {

      // l is the agreed character to start the standard 5pt LRF reading
      // e is the character to start an odometry reading  
    case 'e':

      // predetermined acknowledge char which motherboard requires to accept data
      Serial.write('!');

      // read encoder, and set status and valid bits
      enc1= roboclaw.ReadEncM1(address, &status, &valid);
      if(valid) {

        // prevents underflow, since we are only looking for the relative change in enc cts and reseting it every loop,
        // if the enc subtracts from 0, it will end up overflowing the 16bit variable as well as being incorrect for our 
        // incremental case, for example an enc value reported as 4294967290 will be converted to -5 which is the true
        // relative change from 0 as we expect it.
        if (enc2 > 2^16) {
          enc2 = enc2 - 2^32 + 1;
        }        

        // 1470cts/rev, 2.0944ft/rev, 3.2ft/map unit leads to 0.00044524 map units/encoder ct, or 44524 map units E-8/enc ct
        Dright = enc1*44524; 

        ///* debug prints
        Serial.print("Encoder1:");
        Serial.print(enc1,DEC);
        Serial.print(" ");
        Serial.print(Dright,DEC);
        Serial.println(" ");
        //*/
      }

      // read encoder 2, and set status and valid bits
      enc2= roboclaw.ReadEncM2(address, &status, &valid);
      if(valid) {

        // prevents underflow, since we are only looking for the relative change in enc cts and reseting it every loop,
        // if the enc subtracts from 0, it will end up overflowing the 16bit variable as well as being incorrect for our 
        // incremental case, for example an enc value reported as 4294967290 will be converted to -5 which is the true
        // relative change from 0 as we expect it.
        if (enc2 > 2^16) {
          enc2 = enc2 - 2^32 + 1;
        }

        // 1470cts/rev, 2.0944ft/rev, 3.2ft/map unit leads to 0.00044524 map units/encoder ct, or 44524 map units E-8/enc ct
        Dleft = enc2*44524;

        ///* debug prints
        Serial.print("Encoder2:");
        Serial.print(enc2,DEC);
        Serial.print(" ");
        Serial.print(Dleft,DEC);
        Serial.println(" ");
        //*/
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

      ///* debug prints
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
      //*/

      // iterate the transform variables
      x=x1;
      y=y1;
      phi=phi1;

      // reset the encoders so that we are recieving incremental change in encoder cts
      roboclaw.ResetEncoders(address);

      break;

    default:

      // If the character recieved is not an l or e as designed
      Serial.println("default");
    }
  }
  delay(100);
}


