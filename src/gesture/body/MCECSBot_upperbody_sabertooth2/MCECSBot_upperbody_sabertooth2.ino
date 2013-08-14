#include <SoftwareSerial.h>

#define SABER_TX_PIN               18
#define SABER_RX_PIN               19
#define SABER_BAUDRATE             9600 // Set to 9600/2400 through Sabertooth dip switches
#define SABER_MOTOR1_FULL_FORWARD  127
#define SABER_MOTOR1_FULL_REVERSE  1
#define SABER_MOTOR2_FULL_FORWARD  255
#define SABER_MOTOR2_FULL_REVERSE  128
#define SABER_ALL_STOP             0
#define SABER_MOTOR1_STOP          64
#define SABER_MOTOR2_STOP          192

SoftwareSerial SaberSerial = SoftwareSerial( SABER_RX_PIN, SABER_TX_PIN );

int Actuator1 = 8;    // selects the input pins for the actuator potentiometer
int Actuator2 = 9;    // selects the input pins for the actuator potentiometer
int Actuator3 = 10;    // selects the input pins for the actuator potentiometer
int Actuator4 = 11;    // selects the input pins for the actuator potentiometer

int Actuator1Value= 0;        // places to store actuator loaction values
int Actuator2Value= 0;
int Actuator3Value= 0;
int Actuator4Value= 0;

//setting up select pins. 52 in motor controller 1, 53 is motor controller 2

int MC1 = 46;
int MC2 = 47;

int Max = 150;
int MaxMid = 325;
int Mid = 500;
int MinMid = 675;
int Min = 875;

int Margin = 10;       // Margin for Mid

int FinalPosition1;
int FinalPosition2;
int FinalPosition3;
int FinalPosition4;

int motion1;
int motion2;
int motion3;
int motion4;

const byte ledPin = 13;      // the pin that the LED is attached to
byte incomingByte;      // a variable to read incoming serial data into
boolean ledState = false;

void initSabertooth(void)        // Init software UART to communicate
{ 
  Serial.begin(9600);             // with the Sabertooth 2x5
  pinMode( SABER_TX_PIN, OUTPUT );
  SaberSerial.begin( SABER_BAUDRATE );  
  //delay( 2000 );                // 2 second time //delay for the Sabertooth to init
  pinMode(MC1, OUTPUT);        // Send full stop command 
  pinMode(MC2, OUTPUT);        //SaberSerial.write(byte(SABER_ALL_STOP));
  //setting up pinmodes for select pins
}

void setup( )
{
 initSabertooth();

}


void loop(void)
{ 
    if (Serial.available() > 0) {
      // read the oldest byte in the serial buffer:
      incomingByte = Serial.read();
      Serial.print("Key pressed: ");
      Serial.println(incomingByte);
      // if it's a capital H (ASCII 72), turn on the LED:
      if (incomingByte == 'H') {
        ledState = HIGH;
        Serial.println("Start of Motion Allup");
        Allup();
        Serial.println("End of Motion");       
      } 
      // if it's an L (ASCII 76) turn off the LED:
      if (incomingByte == 'L') {
        ledState = LOW;
        Serial.println("Start of Motion Alldown");
        Alldown();
        Serial.println("End of Motion"); 
      }
      
      if (incomingByte == 'J') {        
        Serial.println("Start of Motion Tilt Right");
        Bend34();
        Serial.println("End of Motion"); 
      }
      
      if (incomingByte == 'K') {        
        Serial.println("Start of Motion Tilt Left");
        Bend12();
        Serial.println("End of Motion"); 
      }     
      
      if (incomingByte == 'F') {        
        Serial.println("Start of Motion Tilt Forward");
        Bend41();
        Serial.println("End of Motion"); 
      }
      
      if (incomingByte == 'B') {        
        Serial.println("Start of Motion  Tilt Backward");
        Bend23();
        Serial.println("End of Motion"); 
      }
      
      if (incomingByte == 'h') {
        Serial.print("Start tilt back right: ");
        //Middle3();
        Middle1();
      }
      
      if (incomingByte == 'j') {
        Serial.print("Start tilt front right: ");
        //Middle3();
        Middle2();
      }
      
      if (incomingByte == 'k') {
        Serial.print("Start tilt front left: ");
        //Middle3();
        Middle3();
      }
      
      if (incomingByte == 'l') {
        Serial.print("Start tilt back left: ");
        //Middle3();
        Middle4();
      }
      
      if (incomingByte == 'q') {
        Serial.println("Start tilt mid left: ");
        //Middle3();
        Bend12m();
      }
      if (incomingByte == 'w') {
        Serial.println("Start tilt mid right: ");
        //Middle3();
        Bend34m();
      }
      if (incomingByte == 'e') {
        Serial.println("Start tilt mid fwd: ");
        //Middle3();
        Bend23m();
      }
      if (incomingByte == 'r') {
        Serial.println("Start tilt mid back: ");
        //Middle3();
        Bend14m();
      }
      
      if (incomingByte == 'R') {        
        Serial.println("Start of Motion Alldown");
        Bend34();
        Bend41();
        Bend12();
        Bend23();
        Alldown();
        Serial.println("End of Motion"); 
      }
      
      if (incomingByte == 'T') {        
        Serial.println("Start of Motion Alldown");
        Bend12();
        Bend41();
        Bend34();
        Bend23();
        Alldown();
        Serial.println("End of Motion"); 
      }
      //digitalWrite(ledPin, ledState);
    }
    /*Serial.println("Start of Motion");         
    Motion();
    Serial.println("End of Motion");  */
 
}


void Motion()
{
  
        motion1=analogRead(4);
        motion2=analogRead(5);
        motion3=analogRead(6);
        motion4=analogRead(7);
  Serial.print("Motion1: ");
  Serial.println(motion1);  
    Serial.print("Motion2: ");
  Serial.println(motion2);  
    Serial.print("Motion3: ");
  Serial.println(motion3);  
    Serial.print("Motion4: ");
  Serial.println(motion4);  
  
  /*if(motion1 >= 1000){        //Task one 4
  Serial.println("Allup!");
  Allup();

  delay(5000);
  }
  
  if(motion2 >= 1000){        //Task two 5
  Serial.println("Alldown!");
  Alldown();

  delay(5000);
  }
  
  if(motion3 >= 1000){        //Task three 6
  Serial.println("Bend12");
  Bend12();

  delay(5000);
  }
  
  if(motion4 >= 1000){        //Task four    7  
  Serial.println("Bend23");
  Bend23();
  delay(5000);
  }
  
*/
 
  Allup();
//
  Alldown();

  AllupMid(MaxMid);
  AlldownMid(MinMid);
  AllupMid(MaxMid);
  AlldownMid(MinMid);
  AllupMid(MaxMid);
  AlldownMid(MinMid);
  
  Alldown();
/*
  Minimum1();
  Minimum2();
  Minimum3();
  Minimum4();
// 
  Maximum1();
  Maximum2();
  Maximum3();
  Maximum4();

// 
  Bend12();
// 
  Middle1();
  Middle2();
  Middle3();
  Middle4();
// 
  Minimum1();
  Minimum2();
  Minimum3();
  Minimum4();
// 
  Middle4();
  Middle3();
  Middle2();
  Middle1();
// 
// 
  Allup();
// 
  Minimum1();
  Minimum2();
  Maximum1();
  Minimum3();
  Maximum2();
  Minimum4();
  Maximum3();
  Maximum4();
// 
  Bend23();
*/
 
 
 
 
 
}

void Off()
{
    digitalWrite(MC1, LOW);                       // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
}

////  ----------------------------------------------------------------------------- //
////  ----------------------------------------------------------------------------- //
////  ----------------------------------------------------------------------------- //
//
void Extreme1()
{ 
//  ----------------------------------------------------------------------------
//          Extreme position of Actuator I
//  ----------------------------------------------------------------------------
  if (Actuator1Value<= Max)
  {
    Serial.println("-- Actuator I at 0% --");
    digitalWrite(MC1, HIGH);                       // SaberTooth 1 selection
    digitalWrite(MC2, LOW);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR1_STOP));    // All stop     
  } 
 
  if (Actuator1Value>= Min)
  {
    Serial.println("-- Actuator I at 100% --");
    digitalWrite(MC1, HIGH);                       // SaberTooth 1 selection
    digitalWrite(MC2, LOW);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR1_STOP));    // All stop     
  } 
//  ----------------------------------------------------------------------------- 
}

 
 
//  ----------------------------------------------------------------------------
//          Position for Actuator I
//  ----------------------------------------------------------------------------
 
  void Maximum1()
  {
    Actuator1Value= analogRead(Actuator1);
    while (Actuator1Value > Max)
    {
    Actuator1Value= analogRead(Actuator1);
    //Serial.println(Actuator1Value);
    //Serial.println("ppppppppppppppppp");
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
    }
    Extreme1();   
    Serial.println("Actuator I at Maximum");   
  }
 
 
  void Minimum1()
  {
    while (Actuator1Value < Min)
    {
    Actuator1Value= analogRead(Actuator1);
    //Serial.println(Actuator1Value);
    //Serial.println("55555555555555555555555555555");
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    Extreme1();     
    Serial.println("Actuator I at Minimum");   
  }
   

  void Middle1()
  {
    if (Actuator1Value < Mid)                    // Above Mid
    {
    Serial.println("Above middle");
    //delay(2000);
    while (Actuator1Value < Mid+10)
      {
      Serial.println("Above middle");
      Actuator1Value= analogRead(Actuator1);
      Serial.println(Actuator1Value);
      digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
      digitalWrite(MC2, LOW);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
      }
    }
   
    else                   // Below Mid
    {
    Serial.println("below middle");
    ////delay(2000);
    while (Actuator1Value > Mid-10)
      {
      Serial.println("below middle-------------");
      Actuator1Value= analogRead(Actuator1);
      Serial.println(Actuator1Value);
      digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
      digitalWrite(MC2, LOW);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
      }
    }    
    SaberSerial.write(byte(SABER_MOTOR1_STOP));
    Serial.println("Actuator I at Middle");  
    //delay(5000);
   
  }
 
 
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
 
 void Extreme2( void )
{ 
//  ----------------------------------------------------------------------------
//          Extreme position of Actuator II
//  ----------------------------------------------------------------------------
  if (Actuator2Value<= Max)
  {
    Serial.println("-- Actuator I at 0% --");
    digitalWrite(MC1, HIGH);                       // SaberTooth 1 selection
    digitalWrite(MC2, LOW);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR2_STOP));    // All stop     
  } 
 
  if (Actuator2Value>= Min)
  {
    Serial.println("-- Actuator I at 100% --");
    digitalWrite(MC1, HIGH);                       // SaberTooth 1 selection
    digitalWrite(MC2, LOW);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR2_STOP));    // All stop     
  } 
//  ----------------------------------------------------------------------------- 
}
 
//------------------------------------------------------------------------------
//          Position for Actuator II
//  ----------------------------------------------------------------------------
void Maximum2()
  {
    Actuator2Value= analogRead(Actuator2);
    while (Actuator2Value > Max)
    {
    Actuator2Value= analogRead(Actuator2);
    Serial.println(Actuator2Value);
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
    }
    Extreme2();   
    Serial.println("Actuator II at Maximum");   
  }
 
 
  void Minimum2()
  {
    while (Actuator2Value < Min)
    {
    Actuator2Value= analogRead(Actuator2);
    Serial.println(Actuator2Value);
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    Extreme2();     
    Serial.println("Actuator II at Minimum");   
  }
   

  void Middle2()
  {
    if (Actuator2Value < Mid)                    // Above Mid
    {
    Serial.println("Above middle");
    //delay(2000);
    while (Actuator2Value < Mid+10)
      {
      Serial.println("Above middle");
      Actuator2Value= analogRead(Actuator2);
      Serial.println(Actuator2Value);
      digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
      digitalWrite(MC2, LOW);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
      }
    }
   
    else                   // Below Mid
    {
    Serial.println("below middle");
    //delay(2000);
    while (Actuator2Value > Mid-10)
      {
      Serial.println("below middle-------------");
      Actuator2Value= analogRead(Actuator2);
      Serial.println(Actuator2Value);
      digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
      digitalWrite(MC2, LOW);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
      }
    }    
    SaberSerial.write(byte(SABER_MOTOR2_STOP));
    Serial.println("Actuator II at Middle");  
    //delay(5000);
   
  }
 
 
  //---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
 
  void Extreme3( void )
{ 
//  ----------------------------------------------------------------------------
//          Extreme position of Actuator III
//  ----------------------------------------------------------------------------
  if (Actuator3Value <= Max)
  {
    Serial.println("-- Actuator I at 0% --");
    digitalWrite(MC1, LOW);                       // SaberTooth 1 selection
    digitalWrite(MC2, HIGH);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR1_STOP));    // All stop     
  } 
 
  if (Actuator3Value >= Min)
  {
    Serial.println("-- Actuator I at 100% --");
    digitalWrite(MC1, LOW);                       // SaberTooth 1 selection
    digitalWrite(MC2, HIGH);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR1_STOP));    // All stop     
  } 
//  ----------------------------------------------------------------------------- 
}

//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
 
  //------------------------------------------------------------------------------
//          Position for Actuator III
//  ----------------------------------------------------------------------------
void Maximum3()
  {
    Actuator3Value= analogRead(Actuator3);
    while (Actuator3Value > Max)
    {
    Actuator3Value= analogRead(Actuator3);
    Serial.println(Actuator3Value);
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
    }
    Extreme3();   
    Serial.println("Actuator III at Maximum");   
  }
 
 
  void Minimum3()
  {
    while (Actuator3Value < Min)
    {
    Actuator3Value= analogRead(Actuator3);
    Serial.println(Actuator3Value);
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    Extreme3();     
    Serial.println("Actuator III at Minimum");   
  }
   

  void Middle3()
  {
    if (Actuator3Value < Mid)                    // Above Mid
    {
    Serial.println("Above middle");
    //delay(2000);
    while (Actuator3Value < Mid+10)
      {
      Serial.println("Above middle");
      Actuator3Value= analogRead(Actuator3);
      Serial.println(Actuator3Value);
      digitalWrite(MC1, LOW);            // SaberTooth 2 selection
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
      }
    }
   
    else                   // Below Mid
    {
    Serial.println("below middle");
    //delay(2000);
    while (Actuator3Value > Mid-10)
      {
      Serial.println("below middle-------------");
      Actuator3Value= analogRead(Actuator3);
      Serial.println(Actuator3Value);
      digitalWrite(MC1, LOW);            // SaberTooth 2 selection
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
      }
    }    
    SaberSerial.write(byte(SABER_MOTOR1_STOP));
    Serial.println("Actuator III at Middle");  
    //delay(5000);
   
  }
 
 // -----------------------------------------------------------------------------
 // -----------------------------------------------------------------------------
 
   void Extreme4( void )
{ 
//  ----------------------------------------------------------------------------
//          Extreme position of Actuator IV
//  ----------------------------------------------------------------------------
  if (Actuator4Value <= Max)
  {
    Serial.println("-- Actuator I at 0% --");
    digitalWrite(MC1, LOW);                         // SaberTooth 1 selection
    digitalWrite(MC2, HIGH);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR2_STOP));     // All stop     
  } 
 
  if (Actuator4Value >= Min)
  {
    Serial.println("-- Actuator I at 100% --");
    digitalWrite(MC1, LOW);                         // SaberTooth 1 selection
    digitalWrite(MC2, HIGH);                        // SaberTooth 2 selection
    SaberSerial.write(byte(SABER_MOTOR2_STOP));     // All stop     
  } 
//  ----------------------------------------------------------------------------- 
}
 
  //------------------------------------------------------------------------------
//          Position for Actuator IV
//  ----------------------------------------------------------------------------
void Maximum4()
  {
    Actuator4Value= analogRead(Actuator4);
    while (Actuator4Value > Max)
    {
    Actuator4Value= analogRead(Actuator4);
    Serial.println(Actuator4Value);
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
    }
    Extreme4();   
    Serial.println("Actuator IV at Maximum");   
  }
 
 
  void Minimum4()
  {
    while (Actuator4Value < Min)
    {
    Actuator4Value= analogRead(Actuator4);
    Serial.println(Actuator4Value);
    Serial.println("999999999");
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    Extreme4();     
    Serial.println("Actuator IV at Minimum");   
  }
   

  void Middle4()
  {
    if (Actuator4Value < Mid)                    // Above Mid
    {
    Serial.println("Above middle");
    //delay(2000);
    while (Actuator4Value < Mid+10)
      {
      Serial.println("Above middle");
      Actuator4Value= analogRead(Actuator4);
      Serial.println(Actuator4Value);
      digitalWrite(MC1, LOW);            // SaberTooth 2 selection
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
      }
    }
   
    else                   // Below Mid
    {
    Serial.println("below middle");
    //delay(2000);
    while (Actuator4Value > Mid-10)
      {
      Serial.println("below middle-------------");
      Actuator4Value= analogRead(Actuator4);
      Serial.println(Actuator4Value);
      digitalWrite(MC1, LOW);            // SaberTooth 2 selection
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
      }
    }    
    SaberSerial.write(byte(SABER_MOTOR2_STOP));
    Serial.println("Actuator IV at Middle");  
    //delay(5000);
   
  }
 
 
  //--------------------------------------------------------------------------
  //--------------------------------------------------------------------------
  //--------------------------------------------------------------------------
 
  void Allup()
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    while (Actuator1Value > Max || Actuator2Value > Max || Actuator3Value > Max || Actuator4Value > Max)
    {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    /*Serial.println(Actuator1Value);
    Serial.println(Actuator2Value);
    Serial.println(Actuator3Value);
    Serial.println(Actuator4Value);*/

    if ( Actuator1Value >= Max)
    {
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
    }
   
    if ( Actuator2Value >= Max)
    {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
    }
    if ( Actuator3Value >= Max)
    {   
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
    }
    if ( Actuator4Value >= Max)
    {
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
    }

   
   // Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    }  
    Serial.println("All Actuator at Maximum");   
  }
 
 
 
  void Alldown()
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    while (Actuator1Value < Min || Actuator2Value < Min || Actuator3Value < Min || Actuator4Value < Min)
    {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    /*Serial.println(Actuator1Value);
    Serial.println(Actuator2Value);
    Serial.println(Actuator3Value);
    Serial.println(Actuator4Value);*/

    if ( Actuator1Value <= Min)
    {
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
   
    if ( Actuator2Value <= Min)
    {   
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ( Actuator3Value <= Min)
    {   
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ( Actuator4Value <= Min)
    {
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }

   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    }  
    Serial.println("All Actuator at Minimum");   
  }
 
 
 
 
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
 
 void Bend12()
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    while (Actuator1Value > Max || Actuator2Value > Max || Actuator3Value < Min || Actuator4Value < Min)
    {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        if ( Actuator1Value >= Max)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value >= Max)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }

        if (Actuator3Value <= Min)
        {
        Actuator3Value= analogRead(Actuator3);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
        }
   
        if (Actuator4Value <= Min)
        {
        Actuator4Value= analogRead(Actuator4);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    }  
    Serial.println("Bend 12");   
  }
  
  void Bend12(int pos)
 {
    byte posbyte = (byte) pos;
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    //while (Actuator1Value > Max || Actuator2Value > Max || Actuator3Value < MaxMid || Actuator4Value < MaxMid)
    while (Actuator3Value < Max && Actuator4Value < Max)
    {
        Actuator1Value= analogRead(Actuator1);
        Actuator2Value= analogRead(Actuator2);
        Actuator3Value= analogRead(Actuator3);
        Actuator4Value= analogRead(Actuator4);
        
        Serial.println(Actuator1Value);
        Serial.println(Actuator2Value);
        Serial.println(Actuator3Value);
        Serial.println(Actuator4Value);

        if ( Actuator1Value >= Max)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value >= Max)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }

        if (Actuator3Value < Max)
        {
          Serial.println("Actuator3 > MaxMid");
          Actuator3Value= analogRead(Actuator3);
          digitalWrite(MC1, LOW);            // SaberTooth 2 selection
          digitalWrite(MC2, HIGH);
          SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
        }
   
        if (Actuator4Value < Max)
        {
          Serial.println("Actuator4 > MaxMid");
          Actuator4Value= analogRead(Actuator4);
          digitalWrite(MC1, LOW);            // SaberTooth 2 selection
          digitalWrite(MC2, HIGH);
          SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    }  
    Serial.print("Bend 12 with pos: ");   
    Serial.println(pos);
  }
  
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
 
 void Bend23()
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    while (Actuator1Value < Min || Actuator2Value > Max || Actuator3Value > Max || Actuator4Value < Min)
    {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        if ( Actuator1Value <= Min)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
        }
       
        if ( Actuator2Value >= Max)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }

        if (Actuator3Value >= Max)
        {
        Actuator3Value= analogRead(Actuator3);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
   
        if (Actuator4Value <= Min)
        {
        Actuator4Value= analogRead(Actuator4);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    }  
    Serial.println("Bend 23");   
  }
  
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
 
 void Bend34()
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    while (Actuator1Value < Min || Actuator2Value < Min || Actuator3Value > Max || Actuator4Value > Max)
    {
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        if ( Actuator1Value <= Min)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
        }
       
        if ( Actuator2Value <= Min)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }

        if (Actuator3Value >= Max)
        {
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
   
        if (Actuator4Value >= Max)
        {
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }
        
        Actuator1Value= analogRead(Actuator1);
        Actuator2Value= analogRead(Actuator2);
        Actuator3Value= analogRead(Actuator3);
        Actuator4Value= analogRead(Actuator4);
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    }  
    Serial.println("Bend 34");   
  }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
 
 void Bend41()
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    while (Actuator1Value > Max || Actuator2Value < Min || Actuator3Value < Min || Actuator4Value > Max)
    {
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        if ( Actuator1Value >= Max)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value <= Min)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }

        if (Actuator3Value <= Min)
        {
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
        }
   
        if (Actuator4Value >= Max)
        {
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }
        
        Actuator1Value= analogRead(Actuator1);
        Actuator2Value= analogRead(Actuator2);
        Actuator3Value= analogRead(Actuator3);
        Actuator4Value= analogRead(Actuator4);
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    }  
    Serial.println("Bend 41");   
  }

 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
 void AllupMid(int val)
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    while (Actuator1Value > val || Actuator2Value > val || Actuator3Value > val || Actuator4Value > val)
    {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    Serial.println(Actuator1Value);
    Serial.println(Actuator2Value);
    Serial.println(Actuator3Value);
    Serial.println(Actuator4Value);

    if ( Actuator1Value >= Max)
    {
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
    }
   
    if ( Actuator2Value >= Max)
    {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
    }
    if ( Actuator3Value >= Max)
    {   
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
    }
    if ( Actuator4Value >= Max)
    {
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
    }

   
   // Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    }  
    Serial.println("All Actuator at Maximum");   
  }
  
  void AlldownMid(int val)
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    while (Actuator1Value < val || Actuator2Value < val || Actuator3Value < val || Actuator4Value < val)
    {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    Serial.println(Actuator1Value);
    Serial.println(Actuator2Value);
    Serial.println(Actuator3Value);
    Serial.println(Actuator4Value);

    if ( Actuator1Value <= Min)
    {
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
   
    if ( Actuator2Value <= Min)
    {   
    digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
    digitalWrite(MC2, LOW);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ( Actuator3Value <= Min)
    {   
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ( Actuator4Value <= Min)
    {
    digitalWrite(MC1, LOW);            // SaberTooth 2 selection
    digitalWrite(MC2, HIGH);
    SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }

   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    }  
    Serial.println("All Actuator at Minimum");   
  }
 
 void Bend12x(int Actuator1Max)
 {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    while (Actuator1Value > Max || Actuator2Value > Max || Actuator3Value < Min || Actuator4Value < Min)
    {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        if ( Actuator1Value >= Max)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value >= Max)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }

        if (Actuator3Value <= Min)
        {
        Actuator3Value= analogRead(Actuator3);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
        }
   
        if (Actuator4Value <= Min)
        {
        Actuator4Value= analogRead(Actuator4);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    }  
    Serial.println("Bend 12");   
  }
  
  /*void Middle3()
  {
    if (Actuator3Value < Mid)                    // Above Mid
    {
    Serial.println("Above middle");
    //delay(2000);
    while (Actuator3Value < Mid+10)
      {
      Serial.println("Above middle");
      Actuator3Value= analogRead(Actuator3);
      Serial.println(Actuator3Value);
      digitalWrite(MC1, LOW);            // SaberTooth 2 selection
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
      }
    }
   
    else                   // Below Mid
    {
    Serial.println("below middle");
    //delay(2000);
    while (Actuator3Value > Mid-10)
      {
      Serial.println("below middle-------------");
      Actuator3Value= analogRead(Actuator3);
      Serial.println(Actuator3Value);
      digitalWrite(MC1, LOW);            // SaberTooth 2 selection
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
      }
    }    
    SaberSerial.write(byte(SABER_MOTOR1_STOP));
    Serial.println("Actuator III at Middle"); */
  
  void Bend12m()
  {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    //while ( Actuator3Value < Mid+10 || Actuator4Value < Mid+10)
    //{
      
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        /*if ( Actuator1Value >= Mid)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value >= Mid)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }*/

        if ((Actuator1Value <= Mid + 10) && (Actuator2Value <= Mid + 10))
          {
            Serial.println(Mid);
          while ((Actuator1Value < Mid+10) && (Actuator2Value < Mid+10))
          {
            Actuator1Value= analogRead(Actuator1);
            Actuator2Value= analogRead(Actuator2);
            Serial.println(Actuator1Value);
            Serial.println(Actuator2Value);
            digitalWrite(MC1, HIGH);            // SaberTooth 2 selection
            digitalWrite(MC2, LOW);
            SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
            SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
            //SaberSerial.write(byte(64));
            //SaberSerial.write(byte(192));
            digitalWrite(MC1, LOW);
            digitalWrite(MC2, HIGH);
            SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
            SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
          }
          digitalWrite(MC1, HIGH);            // SaberTooth 2 selection
          digitalWrite(MC2, LOW);
          SaberSerial.write(byte(SABER_MOTOR1_STOP));
          SaberSerial.write(byte(SABER_MOTOR2_STOP));
          digitalWrite(MC1, LOW);            // SaberTooth 2 selection
          digitalWrite(MC2, HIGH);
          SaberSerial.write(byte(SABER_MOTOR1_STOP));
          SaberSerial.write(byte(SABER_MOTOR2_STOP));
        }
   
        /*if (Actuator4Value <= Mid + 10 )
        {
        Actuator4Value= analogRead(Actuator4);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }*/
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    //}  
    Serial.println("Bend 12m");   
  }
  
  void Bend34m()
  {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    //while ( Actuator3Value < Mid+10 || Actuator4Value < Mid+10)
    //
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        /*if ( Actuator1Value >= Mid)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value >= Mid)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }*/

        if ((Actuator3Value <= Mid + 10) && (Actuator4Value <= Mid + 10))
          {
            Serial.println(Mid);
          while ((Actuator3Value < Mid+10) && (Actuator4Value < Mid+10))
          {
            Actuator3Value= analogRead(Actuator3);
            Actuator4Value= analogRead(Actuator4);
            Serial.println(Actuator3Value);
            Serial.println(Actuator4Value);
            digitalWrite(MC1, LOW);            // SaberTooth 2 selection
            digitalWrite(MC2, HIGH);
            SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
            SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
          }
          SaberSerial.write(byte(SABER_MOTOR1_STOP));
          SaberSerial.write(byte(SABER_MOTOR2_STOP));
        }
   
        /*if (Actuator4Value <= Mid + 10 )
        {
        Actuator4Value= analogRead(Actuator4);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }*/
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    //}  
    Serial.println("Bend 34m");   
  }
  
  void Bend23m()
  {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    //while ( Actuator3Value < Mid+10 || Actuator4Value < Mid+10)
    //
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        /*if ( Actuator1Value >= Mid)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value >= Mid)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }*/

        if ((Actuator2Value <= Mid + 10) && (Actuator3Value <= Mid + 10))
          {
            Serial.println(Mid);
          while ((Actuator2Value < Mid+10) && (Actuator3Value < Mid+10))
          {
            Actuator2Value= analogRead(Actuator2);
            Actuator3Value= analogRead(Actuator3);
            Serial.println(Actuator2Value);
            Serial.println(Actuator3Value);
            digitalWrite(MC1, LOW);            // SaberTooth 2 selection
            digitalWrite(MC2, HIGH);
            SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
            digitalWrite(MC1, HIGH);            // SaberTooth 2 selection
            digitalWrite(MC2, LOW);
            SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
          }
          digitalWrite(MC1, LOW);            // SaberTooth 2 selection
          digitalWrite(MC2, HIGH);
          SaberSerial.write(byte(SABER_MOTOR1_STOP));
          digitalWrite(MC1, HIGH);            // SaberTooth 2 selection
          digitalWrite(MC2, LOW);
          SaberSerial.write(byte(SABER_MOTOR2_STOP));
        }
   
        /*if (Actuator4Value <= Mid + 10 )
        {
        Actuator4Value= analogRead(Actuator4);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }*/
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    //}  
    Serial.println("Bend 23m");   
  }
  
  void Bend14m()
  {
    Actuator1Value= analogRead(Actuator1);
    Actuator2Value= analogRead(Actuator2);
    Actuator3Value= analogRead(Actuator3);
    Actuator4Value= analogRead(Actuator4);
    
    //while ( Actuator3Value < Mid+10 || Actuator4Value < Mid+10)
    //
    
//    Serial.println(Actuator1Value);
//    Serial.println(Actuator2Value);
//    Serial.println(Actuator3Value);
//    Serial.println(Actuator4Value);

        /*if ( Actuator1Value >= Mid)
        {
        digitalWrite(MC1, HIGH);            // SaberTooth 1  selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));
        }
       
        if ( Actuator2Value >= Mid)
        {   
        digitalWrite(MC1, HIGH);            // SaberTooth 1 selection
        digitalWrite(MC2, LOW);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));
        }*/

        if ((Actuator1Value <= Mid + 10) && (Actuator4Value <= Mid + 10))
          {
            Serial.println(Mid);
          while ((Actuator1Value < Mid+10) && (Actuator4Value < Mid+10))
          {
            Actuator1Value= analogRead(Actuator1);
            Actuator4Value= analogRead(Actuator4);
            Serial.println(Actuator1Value);
            Serial.println(Actuator4Value);
            digitalWrite(MC1, LOW);            // SaberTooth 2 selection
            digitalWrite(MC2, HIGH);
            SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
            digitalWrite(MC1, HIGH);            // SaberTooth 2 selection
            digitalWrite(MC2, LOW);
            SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
          }
          digitalWrite(MC1, LOW);            // SaberTooth 2 selection
          digitalWrite(MC2, HIGH);
          SaberSerial.write(byte(SABER_MOTOR2_STOP));
          digitalWrite(MC1, HIGH);            // SaberTooth 2 selection
          digitalWrite(MC2, LOW);
          SaberSerial.write(byte(SABER_MOTOR1_STOP));
        }
   
        /*if (Actuator4Value <= Mid + 10 )
        {
        Actuator4Value= analogRead(Actuator4);
        digitalWrite(MC1, LOW);            // SaberTooth 2 selection
        digitalWrite(MC2, HIGH);
        SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
        }*/
 
   
    //Extreme1(); 
    //Extreme2(); 
    //Extreme3(); 
    //Extreme4();
    
    //}  
    Serial.println("Bend 14m");   
  }
