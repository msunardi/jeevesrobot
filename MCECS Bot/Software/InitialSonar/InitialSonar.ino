
//Digital pin 7 for reading in the pulse width from the MaxSonar device.
//This variable is a constant because the pin will not change throughout execution of this code.
int sonar;

//variables needed to store values
long pulse;

void setup() {
  //This opens up a serial connection to shoot the results back to the PC console
  Serial.begin(9600);

}
void loop() {
  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);
  pinMode(7, INPUT);
  pinMode(6, INPUT);
  pinMode(5, INPUT);
  pinMode(4, INPUT);
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  delay(350);
  
  for( int sonar = 2; sonar <= 13; sonar ++){
    pulse = pulseIn(sonar, HIGH);
    delay(50);
    pulse = pulse/147;
    Serial.print(" sonar");
    Serial.print(sonar);
    Serial.print(": ");
    Serial.print(pulse);
  }
  
  Serial.println();
}


