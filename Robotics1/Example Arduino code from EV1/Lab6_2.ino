/* Jesse Adams and Jeff Anderson
ECE 410/510 
Lab 6 Part 2 

The following code reads the Hall sensor on Arduino pin 2
and drives its value to the LED and meter on pin 10. */


/******** Initializing the variables **********/ 
int aDirPin = 12;   // power pole A direction
int aEnbPin = 11;   // power pole A enable
int ledPin = 10;    // LED
int potPin = A0;    // potentiometer analog input
int count = 0;      // counter used in for loop to adjust duty ratio
int Hallsensor = 2; // Hall sensor for pin 2

int rot_a, rot_b; 
float potVal, duty_cycle, Hall_input;
unsigned long time_1, time_2, revTime, RPM;


/******** Initializing Arduino Pins **********/
void setup() {
// setup power poles
digitalWrite(aDirPin, HIGH);   // A HIGH
pinMode(aDirPin, OUTPUT);
analogWrite(aEnbPin, HIGH);   // A enabled
pinMode(aEnbPin, OUTPUT);     // setup Potentiometer pi
pinMode(potPin, INPUT);       // setup LED
pinMode(ledPin, OUTPUT);    

Serial.begin(57600);           // initialize baud rate
}

/******** Program **********/
void loop() {                 
potVal = analogRead(potPin);   // assign the potentiometer value to PotVal
duty_cycle = potVal * .249;    // Convert analogRead value to a number 
                               // between 0 and 255. 

Hall_input = digitalRead(Hallsensor);  // Read the Hallsensor pin

if (Hall_input == 0){          // Change values to write to meter.
  Hall_input = 255;
}
else {
  Hall_input = 0;
}

analogWrite(ledPin, Hall_input);    // Write the value of the Hallsensor 
                                    // to meter on pin 10.        
analogWrite(aEnbPin, duty_cycle);   // Write the value of the duty_cycle 
                                    // to the motor. 


/*** print the RPM every second ***/ 
count ++; 

if (count == 1000){
  count = 0; 
  Serial.print("RPM: ");
  Serial.println(RPM);
}
}
