/* Jesse Adams and Jeff Anderson
ECE 410/510 
Lab 6 Part 1 

The following code implements the program from lab 5
using the hardware PWM. */


/******** Initializing the variables **********/ 

int aDirPin = 12;  // power pole A direction
int aEnbPin = 11;  // power pole A enable
int ledPin = 10;   // LED
int potPin = 0;    // potentiometer analog input
int count = 0;     // counter used in for loop to adjust duty ratio
float potVal, duty_cycle;

/******** Initializing Arduino Pins **********/

void setup() {
// setup power poles
pinMode(3, OUTPUT);
pinMode(4, OUTPUT);     // setup Potentiometer pi
pinMode(5, OUTPUT);     // setup LED
pinMode(6, OUTPUT);
pinMode(7, OUTPUT);
pinMode(8, OUTPUT);
pinMode(12, OUTPUT);

Serial.begin(57600);           // initialize baud rate
}
/******** Program **********/

void loop() {                  // switch motor and LED every second
//potVal = analogRead(potPin);   //assign the potentiometer value to PotVal
//duty_cycle = potVal * .249;    //Convert analogRead value to a number 
                               //between 0 and 255. 

analogWrite(11, 200);   //Write the converted value to pin 11
analogWrite(3, 200);   //Write the converted value to pin 11
analogWrite(5, 200);   //Write the converted value to pin 11
analogWrite(6, 200);   //Write the converted value to pin 11
digitalWrite(4, LOW);
digitalWrite(7, LOW);
digitalWrite(8, LOW);
digitalWrite(12, LOW);

/**** Print the values  ****/ 
Serial.println(duty_cycle);
Serial.println(potVal);

}
