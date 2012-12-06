
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
  delay(350);
  Serial.begin(9600);
  AllSonars = true;

}


void loop() {
  
   if (Serial.available() > 0) {
    int incoming = Serial.read();
    Serial.print(incoming);
  
  
    switch (incoming) {

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
      //Serial.print(pulse/147);
      isort(rangevalue,arraysize);
      //printArray(rangevalue,arraysize);
      modE = mode(rangevalue,arraysize);
      //Serial.print("    The mode/median is: ");
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
//Mode function, returning the mode or median.
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


