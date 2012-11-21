#include "BMSerial.h"

BMSerial sensorSerial(6,6);

void setup() {
  Serial.begin(38400);
  sensorSerial.begin(38400);
}

void loop() {
  sensorSerial.println("#AA");
  Serial.print("RState:");
  Serial.print(sensorSerial.readhex(10000),HEX);
  Serial.print(" CState:");
  Serial.print(sensorSerial.readhex(10000),HEX);
  Serial.print(" LState:");
  Serial.println(sensorSerial.readhex(10000),HEX);
  
  delay(100);
}
