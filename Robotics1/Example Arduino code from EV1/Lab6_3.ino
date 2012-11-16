/* Jesse Adams and Jeff Anderson
ECE 410/510 
Lab 6 Part 3 

The following code reads the Hall sensor on Arduino pin 2
and drives its value to the LED and meter on pin 10. */


/******** Initializing the variables **********/ 
int aDirPin = 12;   // power pole A direction
int aEnbPin = 11;   // power pole A enable
int ledPin = 10;    // LED
int potPin = A0;    // potentiometer analog input
int count = 0;      // counter used in for loop to adjust duty ratio
int Hallsensor = 2; // Hall sensor for pin 2; 

int rot_a, rot_b, LedOn, LedOff, count1; 
float potVal, duty_cycle, Hall_input;
float time_1, time_2, revTime,RPM;



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
void loop() { // switch motor and LED every second
  

potVal = analogRead(potPin);   // assign the potentiometer value to PotVal
duty_cycle = potVal * .249;    // Convert analogRead value to a number 
                               // between 0 and 255. 

Hall_input = digitalRead(Hallsensor);  // Read the Hallsensor pin
rot_b = rot_a;                         // Make rotation b = rotation a
rot_a = Hall_input;                    // Make rotation a = Hallsensor

if (rot_a < rot_b){          // If rotation a < b. Falling edge occurred
                             // so do the following. 
  time_2 = time_1;           // save previously saved time
  time_1 = micros();         // get new time. 
  revTime = time_1 - time_2; // Find the difference between the two
  RPM = 60000000.0/revTime;    // Convert that difference to RPM.
 // this code is calculated every rotation, which means the conversion 
 // to RPM above can be made solely through a time conversion.  
}

LedOn = (RPM/4800)*255;

if (LedOn < 256 && duty_cycle < 256){
  analogWrite(ledPin, LedOn);    // Write the value of the Hallsensor 
                                    // to meter on pin 10.        
  analogWrite(aEnbPin, duty_cycle);   // Write the value of the duty_cycle 
}                                    // to the motor. 

count ++; 

if (count == 1000){
  count = 0; 
  Serial.print("Desired RPM: ");
  Serial.println(desiredRPM);
  Serial.print("RPM: ");
  Serial.println(RPM);
  Serial.print("Duty Cycle: ");
  Serial.println(duty_cycle);
}
}
