/*
Read voltage from analog inputs A0 and A1, and print the results 
on the Serial port at 50 Hz. 
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {

  // read the inputs on analog pins 0, 1:
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);

  // Convert 10-bit analog reading to a voltage (0 - 5V):
  float A0_V = a0 * (5.0 / 1023.0);
  float A1_V = a1 * (5.0 / 1023.0); 

  // send
  Serial.print("A0:");
  Serial.print(A0_V, 4);
  Serial.print(", A1:");
  Serial.println(A1_V, 4);
  delay(20); // 50Hz
}
