// Slave Sonar
// Jesse Adams 
#include <Wire.h>
#define arraysize 5 // size of array that stores sonar data

uint8_t pulse1;
uint8_t pulse2;
uint8_t pulse3;
uint8_t pulse4;
uint8_t pulse5;
uint8_t pulse6;
uint8_t pulse7;
uint8_t pulse8;
uint8_t pulse9;
uint8_t pulse10;
uint8_t pulse11;
uint8_t pulse12;

uint8_t sonar1 = 7;
uint8_t sonar2 = 6;
uint8_t sonar3 = 5;
uint8_t sonar4 = 2;
uint8_t sonar5 = 3;
uint8_t sonar6 = 4;
uint8_t sonar7 = 10;
uint8_t sonar8 = 9;
uint8_t sonar9 = 8;
uint8_t sonar10 = 11;
uint8_t sonar11 = 12;
uint8_t sonar12 = 13;
int InterruptPin = A1;

boolean ALERT = false;          // Tells whether object detect. Must "alert" the mega
uint8_t obstacle;               // Holds value for data transmission to mega
uint8_t data_to_send[2];        // In I2C protocol, if sending multiple bytes, they 
                                // must be stored in a uint8_t array
int threshold = 15;             // distance in inches that will trigger robot to stop
int kinectVal = -1;             // initializing value. ie don't do anything until instructed
int rangevalue[arraysize] = { 0, 0, 0, 0, 0 };  // data array for sonars
int slave_address = 2;          // I2C slave address of Arduino

void setup()
{
  Wire.begin(slave_address);      // join i2c bus with address #2
  Wire.onReceive(receiveEvent);   // register event
  Wire.onRequest(requestEvent);   // register event
  Serial.begin(9600);             // start serial for output
  pinMode(InterruptPin, OUTPUT);  // Alert pin for Mega
  digitalWrite(InterruptPin, ALERT);  // Initialize as low 

  delay(250);                    // delay required for sonars

  pulseIn(sonar1, HIGH);         // Each sonar must be initialized with a read of 102ms
  delay(102);                    // this is a calibration reading
  pulseIn(sonar2, HIGH);
  delay(102);
  pulseIn(sonar3, HIGH);
  delay(102);
  pulseIn(sonar4, HIGH);
  delay(102);
  pulseIn(sonar5, HIGH);
  delay(102);
  pulseIn(sonar6, HIGH);
  delay(102);
  pulseIn(sonar7, HIGH);
  delay(102);
  pulseIn(sonar8, HIGH);
  delay(102);
  pulseIn(sonar9, HIGH);
  delay(102);
  pulseIn(sonar10, HIGH);
  delay(102);
  pulseIn(sonar11, HIGH);
  delay(102);
  pulseIn(sonar12, HIGH);
  delay(102); 
  Serial.begin(9600);
}

void loop()
{
 // Code that executes while waiting from I2C communication
 // This commented part is part of Jesse's original code
  if(Serial.available()){                        // If new movement available, retreive.
    kinectVal = Serial.read();   
	Serial.print("Kinectval: ");
	Serial.println(kinectVal);
  }
  else {
    
    if (kinectVal != 4) { // if not 'stop' 
      pulse1 = getSonar(sonar1);
      if(pulse1 < threshold){
          stop_robot(sonar1, pulse1);
          //Serial.println("Sonar 1");
      }
      /*delay(200);
      pulse7 = getSonar(sonar7);
      if(pulse7 < threshold){
          stop_robot(sonar7, pulse7);
          //Serial.println("Sonar 7");
      }*/
      delay(200);
      pulse3 = getSonar(sonar3);
      if(pulse3 < threshold){
          stop_robot(sonar3, pulse3);
          //Serial.println("Sonar 3");
      }
      /*delay(200);
      pulse9 = getSonar(sonar9);
      if(pulse9 < threshold){
          stop_robot(sonar9, pulse9);
          //Serial.println("Sonar 9");
      }*/
      /*delay(200);
      pulse12 = getSonar(sonar12);
      if(pulse12 < threshold){
          stop_robot(sonar12, pulse12);
          //Serial.println("Sonar 12");
      }*/
      /*delay(200);
      pulse6 = getSonar(sonar6);
      if(pulse6 < threshold){
          stop_robot(sonar6, pulse6);
          //Serial.println("Sonar 6");
      }*/
      delay(200);
      pulse10 = getSonar(sonar10);
      if(pulse10 < threshold){
          stop_robot(sonar10, pulse10);
          //Serial.println("Sonar 10");
      }
      /*delay(200);
      pulse4 = getSonar(sonar4);
      if(pulse4 < threshold){
          stop_robot(sonar4, pulse4);
          //Serial.println("Sonar 4");
      }*/
      delay(200);
      pulse2 = getSonar(sonar2);
      if(pulse2 < threshold){
          stop_robot(sonar2, pulse2);
          //Serial.println("Sonar 2");
      }
      /*delay(200);
      pulse8 = getSonar(sonar8);
      if(pulse8 < threshold){
          stop_robot(sonar8, pulse8);
          //Serial.println("Sonar 8");
      }*/
      delay(200);
      pulse11 = getSonar(sonar11);
      if(pulse11 < threshold){
          stop_robot(sonar11, pulse11);
          //Serial.println("Sonar 11");
      }
      /*delay(200);
      pulse5 = getSonar(sonar5);
      if(pulse5 < threshold){
          stop_robot(sonar5, pulse5);
          //Serial.println("Sonar 5");
      }*/
      delay(200);
    }
  }  
  ///// From this point until the end of the loop() is the original code
 
  
  
  switch(kinectVal){
/*
    case 0:                              // case 0: Turn Left 
    
	Serial.println("Case 0");
      pulse3 = getSonar(sonar3);
        if(pulse3 < threshold){
           stop_robot(sonar3, pulse3);
        }

      pulse5 = getSonar(sonar5);
        if(pulse5 < threshold){
           stop_robot(sonar5, pulse5);
         }

      pulse8 = getSonar(sonar8);
        if(pulse8 < threshold){
           stop_robot(sonar8, pulse8);
         }
      pulse11 = getSonar(sonar11);
        if(pulse11 < threshold){
           stop_robot(sonar11, pulse11);
         }

    break;

    case 1:                              // case 1: Turn Right
    Serial.println("Case 1");
      pulse3 = getSonar(sonar3);
        if(pulse3 < threshold){
           stop_robot(sonar3, pulse3);
        }

      pulse5 = getSonar(sonar5);
        if(pulse5 < threshold){
           stop_robot(sonar5, pulse5);
         }

      pulse8 = getSonar(sonar8);
        if(pulse8 < threshold){
           stop_robot(sonar8, pulse8);
         }
      pulse11 = getSonar(sonar11);
        if(pulse11 < threshold){
           stop_robot(sonar11, pulse11);
         }
    break;
*/
    case 2:                              // case 2: Move Forward 
    Serial.println("Case 2");
      pulse1 = getSonar(sonar1);
        if(pulse1 < threshold){
           stop_robot(sonar1, pulse1);
        }
      pulse2 = getSonar(sonar2);   
        if(pulse2 < threshold){
           stop_robot(sonar2, pulse2);
        }
      pulse11 = getSonar(sonar11);
        if(pulse11 < threshold){
           stop_robot(sonar11, pulse11);
        }
      pulse12 = getSonar(sonar12);   
        if(pulse12 < threshold){
           stop_robot(sonar12, pulse12);
        }

    break;

    case 3:                              // case 3: Move Backward 
    Serial.println("Case 3");
      pulse6 = getSonar(sonar6);
           if(pulse6 < threshold){
           stop_robot(sonar6, pulse6);
        }
      //pulse7 = getSonar(sonar7);
      //    if(pulse7 < threshold){
      //     stop_robot(sonar7, pulse7);
      //  }
      pulse8 = getSonar(sonar8);
           if(pulse8 < threshold){
           stop_robot(sonar8, pulse8);
        }
      pulse9 = getSonar(sonar9);
           if(pulse9 < threshold){
           stop_robot(sonar9, pulse9);
        }
    break;

    case 4:
       // Stopped
       Serial.println("Case 4");
       break;
       
    case 2:                              // case 2: Move Forward 
    Serial.println("Case 2");
      pulse1 = getSonar(sonar1);
        if(pulse1 < threshold){
           stop_robot(sonar1, pulse1);
        }
      pulse2 = getSonar(sonar2);   
        if(pulse2 < threshold){
           stop_robot(sonar2, pulse2);
        }
      pulse11 = getSonar(sonar11);
        if(pulse11 < threshold){
           stop_robot(sonar11, pulse11);
        }
      //pulse12 = getSonar(sonar12);   
      //  if(pulse12 < threshold){
      //     stop_robot(sonar12, pulse12);
      //  }

    break;
    
    default:
       Serial.println("Nothing");
	


  }
}

///             Get Sonar Function              ///
/* Programmer passes sonar number into this function
to return sonar reading. i.e. getSonar(sonar3);   */

int getSonar(int sonar) {
      int pulse;
      for(int i = 0 ; i < arraysize; i++){
          pulse = pulseIn(sonar, HIGH);
          delay(48);
          rangevalue[i] = pulse/147;
      }
      isort(rangevalue, arraysize);
      pulse = mode(rangevalue, arraysize);
    return pulse;
}

///////////////   end Sonar Function  ///////////////

///             Stop Robot Function              ///
/* If any sonar detects a reading of less than the 
variable "threshold" it will call this function. 
Starts by setting the Interrupt Pin High to noticy 
the Mega. Then it spins in a while loop, continuously
reading the sonar that detected the obstacle. Once 
the obstacle is removed the loop ends and the function
removes the alert to the Mega by driving the interrupt
pin low. */

void stop_robot(uint8_t sonar, uint8_t obstacle){
  ALERT = HIGH;
  data_to_send[0] = obstacle;
  data_to_send[1] = sonar;
  digitalWrite(InterruptPin, ALERT);
    while(obstacle < threshold){
      data_to_send[0] = obstacle;
      data_to_send[1] = sonar;
      Serial.print("obstacle: ");
      Serial.print(obstacle);
      Serial.print("  sonar: ");
      Serial.print(sonar);            // Debug Block
      Serial.print("  ALERT: ");
      Serial.println(ALERT);
      Serial.print("Data_to_send0: ");
        Serial.println(data_to_send[0]);
        Serial.print("Data_to_send1: ");
        Serial.println(data_to_send[1]);
	  delay(200);
      obstacle = getSonar(sonar);
    }
   ALERT = LOW;
   digitalWrite(InterruptPin, ALERT);
   //data_to_send[0] = 25;
   //data_to_send[1] = 25;
}  

// function that executes whenever data is requested from master
// this function is registered as an event, see setup()   
void requestEvent()
{
  Wire.write(data_to_send, 2);  // respond with message of 2 bytes
                                // as expected by master
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  kinectVal = Wire.read();        // receive byte as an integer
//  Serial.println(kinectVal);          // print the integer
}

// isort is used to sort the array of read values from a sonar //
void isort(int *a, int n){
  for (int i = 1; i < n; ++i){                    
    int j = a[i];                                 
    int k;                          
    for (k = i - 1; (k >= 0) && (j < a[k]); k--){
      a[k + 1] = a[k];
      }
    a[k + 1] = j;
  }
}
//////////////////////     End isort        ////////////////////

///        Mode function, returns the mode or median.        ///
int mode(int *x,int n){
  int i = 0;
  int count = 0;
  int maxCount = 0;
  int mode = 0;
  int bimodal;
  int prevCount = 0;
  while(i<(n-1)){
    prevCount=count;
    count=0;
    while(x[i] == x[i+1]){
      count++;
      i++;
    }
    if(count>prevCount&count>maxCount){
      mode=x[i];
      maxCount=count;
      bimodal=0;
    }
    if(count==0){
      i++;
    }
    if(count==maxCount){      //If the dataset has 2 or more modes.
      bimodal=1;
    }
    if(mode==0||bimodal==1){//Return the median if there is no mode.
      mode=x[(n/2)];
    }
    return mode;
  }
}
////////////////////         end mode       /////////////////////// 
