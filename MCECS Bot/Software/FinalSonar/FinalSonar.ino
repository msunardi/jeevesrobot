
/***********************************************/
/* The following code is the final code for 
/* implementation of the 12 sonars on MCECS Bot.
/* To use this code simply upload and open the 
/* serial monitor. Sonar measurements will be 
/* taken when the user indicates which sonar 
/* should be read. To do this the each sonar is 
/* activated by an ascii input. 1-9 are for sonars
/* 1-9. ! is sonar 10, @ is sonar 11 and # is sonar
/* 12. Send ascii 'a' for the code to read all sonars.*/

#include <SoftwareSerial.h> 


int sonar;
int randoms; 
long pulse;
int arraysize = 5;
int rangevalue[] = { 0, 0, 0, 0, 0};
int modE;
boolean AllSonars;

void setup() {
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
  delay(350);      // delay for sonar initialization
  Serial.begin(9600);
  AllSonars = true; // variable for reading all sonars at once

}


void loop() {
                                      
   if (Serial.available() > 0) {    // read data from computer if available
    int incoming = Serial.read();   // store value in 'incoming'
    Serial.print(incoming);         // print that value to screen
  
  
    switch (incoming) {

      // case 97 is the all sonar read function. It loops through
      // all sonars, reading from each 5 times.
      case 97:                
         sonar = 2;
         AllSonars = true;
         for( int sonar = 2; sonar <= 13; sonar ++){
           for(int i = 0 ; i < arraysize; i ++){
            pulse = pulseIn(sonar, HIGH);
            delay(48);
            rangevalue[i] = pulse/147;
           }
      Serial.print(" sonar");
      Serial.print(sonar-1);
      Serial.print(": ");
      //Serial.print(pulse/147);              //extra print function
      isort(rangevalue,arraysize);
      //printArray(rangevalue,arraysize);    // extra print function
      modE = mode(rangevalue,arraysize);
      //Serial.print("    The mode/median is: ");  // extra print function
      Serial.print(modE);
      Serial.println();
         }
      Serial.println(); 
      break;
      
      
      
      case 49:
         sonar = 2;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
 
 
      case 50:
         sonar = 3;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
        
      case 51:
         sonar = 4;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
        
      case 52:
         sonar = 5;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
        
       case 53:
         sonar = 6;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break; 
        
       case 54:
         sonar = 7;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
        
       case 55:
         sonar = 8;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break; 
        
       case 56:
         sonar = 9;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break; 
              
       case 57:
         sonar = 10;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
        
       case 33:
         sonar = 11;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
        
        case 64:
         sonar = 12;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
        
        case 35:
         sonar = 13;
         for(int i = 0 ; i < arraysize; i ++){
           pulse = pulseIn(sonar, HIGH);
           delay(48);
           rangevalue[i] = pulse/147;
           AllSonars = false;
          }
        break;
    }
}
  
// Becuase of the way the information is printed in the all sonar 
// function, this block is needed to not duplicate the printed 
// information. This block only prints on single sonar readings. 

  if(AllSonars == false){
      Serial.print(" sonar");
      Serial.print(sonar-1);
      Serial.print(": ");
      //Serial.print(pulse/147);
      isort(rangevalue,arraysize);
      //printArray(rangevalue,arraysize);
      modE = mode(rangevalue,arraysize);
      //Serial.print("    The mode/median is: ");
      Serial.print(modE);
      Serial.println(); 
      AllSonars = true;
      
  } 
    
}


void printArray(int *a, int n) {
  for (int i = 0; i < n; i++)
  {
    Serial.print(a[i], DEC);
    Serial.print(' ');
  }
}



// This block of code sorts the sonar readings by value
void isort(int *a, int n){
// *a is an array pointer function
  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}



//Mode function returns the mode or median.
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
    if(count==maxCount){//If the dataset has 2 or more modes.
      bimodal=1;
    }
    if(mode==0||bimodal==1){//Return the median if there is no mode.
      mode=x[(n/2)];
    }
    return mode;
  }
}


