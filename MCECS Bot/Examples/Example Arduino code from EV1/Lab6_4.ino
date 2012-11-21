/* Jesse Adams and Jeff Anderson
ECE 410/510 
Lab 6 Part 3 

The following code determines the motor's RPM using rising edge detection.
RPM is then used to calculate a proportional pulse width modulation value 
to be written to the Vu Meter. 
*/


/******** Initializing the variables **********/ 
int aDirPin = 12;   // power pole A direction
int aEnbPin = 11;   // power pole A enable
int ledPin = 10;    // LED
int potPin = 0;    // potentiometer analog input
int count = 0;      // counter used in for loop to adjust duty ratio
int Hallsensor = 2; // Hall sensor for pin 2

int count1 = 0, counter2 = 0;  //
int countdown = 200;  //countdown for kick-starting the motor
int rot_a;  //
int rot_b;  //
int RPMnew;
int potVal;
int duty_cycle = 0;
int Hall_input, correction;

float time_1;
float time_2;
float conversion = 60000000.0; //conversion factor for microsec to minutes
float RPM, revTime, RPMmeter, desiredRPM;


/******** Initializing Arduino Pins **********/
void setup() {
// setup power poles
digitalWrite(aDirPin, HIGH);   // A HIGH
pinMode(aDirPin, OUTPUT);
analogWrite(aEnbPin, HIGH);   // A enabled
pinMode(aEnbPin, OUTPUT);     // setup Potentiometer pi
pinMode(potPin, INPUT);       // setup LED
pinMode(ledPin, OUTPUT);    

Serial.begin(57600); // initialize baud rate
}

/******** Program **********/
void loop() {
potVal = analogRead(potPin);     // assign the potentiometer value to PotVal
desiredRPM = potVal * 4.8877;    // Convert analogRead value to an RPM 
                                 // value.  

Hall_input = digitalRead(Hallsensor);  // Read the Hallsensor pin
rot_b = rot_a;                         // Make rotation b = rotation a
rot_a = Hall_input;                    // Make rotation a = Hallsensor


/*** RPM Calculation ***/ 
if (rot_a < rot_b){          // If rotation a > b rising edge occurred
                             // so do the following. 
  time_2 = time_1;           // save previously saved time
  time_1 = micros();         // get new time. 
  revTime = time_1 - time_2; // Find the difference between the two.
  RPM = 60000000.0/(revTime);    // Convert that difference to RPM.
 // this code is calculated every rotation, which means the conversion 
 // to RPM above can be made solely through a time conversion. There are
 // 60 million microseconds in 60 seconds. 
}

/*** Increase or Decrease the duty cycle accordingly ***/ 
count1++;
if (count1 == 201){
  count1 = 0;
}
  if (desiredRPM > RPM && duty_cycle < 255 && count1 == 200){
    count1=0;
    duty_cycle++;
  }
  else if (desiredRPM < RPM && duty_cycle > 0 && count1 == 200){
    count1=0;
    duty_cycle--;
  }

if (desiredRPM == 0 && RPM > 0 && count1 == 200) {
  RPM = 0;
  }

analogWrite(aEnbPin, duty_cycle);      //write new duty cycle to the motor
analogWrite(ledPin, duty_cycle);
//if potentiometer is zero and RPM is positive, bring RPM down to zero


//Delay printing variables
count ++;
if (count == 1000) {
  count = 0;
  Serial.print("Desired RPM: ");
  Serial.println(desiredRPM);
  Serial.print("RPM: ");
  Serial.println(RPM);
  Serial.print("Duty Cycle: ");
  Serial.println(duty_cycle);
}
}


