/*
 ReadAnalogPosition
 Reads an analog input on pin 0, 1, 2 and 3, prints the result to the serial monitor 
 */

void setup() {
  Serial.begin(9600);
  //the setup of the input and output ports on the Arduino are missing
  //something like: pinMode(13, Output);
}

void loop() {
  int positionM1 = analogRead(A0);
  int positionM2 = analogRead(A1);
  int positionM3 = analogRead(A3);
  int positionM4 = analogRead(A4);
  Serial.print(positionM1);
  Serial.print("_");
  Serial.print(positionM2);
  Serial.print("_");
  Serial.print(positionM3);
  Serial.print("_");
  Serial.println(positionM4);
  delay(200);
  
  
}


