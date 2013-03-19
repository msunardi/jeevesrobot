#include <SoftwareSerial.h>
/*****************************************************
 * DIP Switches as per the wizard:
 * - NiMh Battery
 * - TTL RS232
 * - Simplified Serial Mode
 * - Two Saberteeth connected
 * - 9600 baudrate
 *
 * Pin 1 - ON
 * Pin 2 - OFF
 * Pin 3 - OFF
 * Pin 4 - OFF
 * Pin 5 - ON
 * Pin 6 - OFF
 * Treating Act 3/4 as if they were facing forward. 
 *
 ****************************************************/

// Labels for use with the Sabertooth 2x5 motor controller

// Digital pin 13 is the serial transmit pin to the 
// Sabertooth 2x5
#define SABER_TX_PIN               18

// NOT USED (but still init'd)
// Digital pin 12 is the serial receive pin from the 
// Sabertooth 2x5
#define SABER_RX_PIN               19

// Set to 9600 through Sabertooth dip switches
#define SABER_BAUDRATE             9600

// Simplified serial Limits for each motor
//64 is soft stop
#define SABER_MOTOR1_FULL_FORWARD  127
#define SABER_MOTOR1_FULL_REVERSE  1
//192 is soft stop
#define SABER_MOTOR2_FULL_FORWARD  255
#define SABER_MOTOR2_FULL_REVERSE  128


// Motor level to send when issuing the full stop command
#define SABER_ALL_STOP             0


SoftwareSerial SaberSerial = SoftwareSerial( SABER_RX_PIN,
                                             SABER_TX_PIN );
                                             
// Setting up the actuator pots, connected to a8-a11:
//analogRead(Actuatorx) reads in the value from the pin. 

int Actuator1 = A8;    // selects the input pins for the actuator potentiometer
int Actuator2 = A9; 
int Actuator3 = A10; 
int Actuator4 = A11; 

// places to store actuator loaction values
int Actuator1Value = 0;
int Actuator2Value = 0;
int Actuator3Value = 0;
int Actuator4Value = 0;
//setting up select pins. 52 in motor controller 1, 53 is motor controller 2

int MC1 = 52;
int MC2 = 53;

   
void initSabertooth( void )
{
  // Init software UART to communicate 
  
  
  
  // with the Sabertooth 2x5
  pinMode( SABER_TX_PIN, OUTPUT );

  SaberSerial.begin( SABER_BAUDRATE );     

  // 2 second time delay for the Sabertooth to init
  delay( 2000 );

  // Send full stop command
    SaberSerial.write(byte(SABER_ALL_STOP));
  
  //setting up pinmodes for select pins
  pinMode(MC1, OUTPUT);
  pinMode(MC2, OUTPUT);
 
  // setting pinmode for analog pins SHOULDN'T NEED THIS
  //pinMode(Actuator4,INPUT);
  /*pinMode(Actuator2,INPUT);
  pinMode(Actuator3,INPUT);
  pinMode(Actuator4,INPUT);
  
  */
}

void setup( )
{
  initSabertooth( );
  
}

//Actuators 4 and 3 will be facing the front of the robot, and dubbed the front actuators
//moving to a nuetral position
void Nuetral( void )
{


  boolean nuetral = false; //all actuators are in nuetral position
  boolean nuetral1 = false; // checking actuator 1 in position
  boolean nuetral2 = false; // checking actuator 2 in position
  boolean nuetral3 = false; // checking actuator 3 in position
  boolean nuetral4 = false; // checking actuator 4 in position
//64 will stop the motor 1
//192 is stop for motor 2
//All MC's active to set a nuetral position  do i need to worry about this? 
// I'm setting the MC's with ever loop check
  while (!nuetral)
  { 
    // Reading in values for actuator position, lower is more extended
    Actuator1Value = analogRead(Actuator1);
    Actuator2Value = analogRead(Actuator2);
    Actuator3Value = analogRead(Actuator3);
    Actuator4Value = analogRead(Actuator4);
    // checking and adjusting actuator 1
    if (Actuator1Value >= 615)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator1Value <= 585)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator1Value > 585) && (Actuator1Value < 615))
     nuetral1 = true;
    
    
    // checking and adjusting actuator 2
    if (Actuator2Value >= 615)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator2Value <= 585)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator2Value > 585) && (Actuator2Value < 615))
     nuetral2 = true;     
     
    
    // checking and adjusting actuator 3
    if (Actuator3Value >= 615)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator3Value <= 585)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator3Value > 585) && (Actuator3Value < 615))
     nuetral3= true;  
    
    
    // checking and adjusting actuator 4
    if (Actuator4Value >= 615)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator4Value <= 585)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator4Value > 585) && (Actuator4Value < 615))
     nuetral4 = true;       
    
    // checking to make sure all actuators are in right position before leaving loop.  
    if (nuetral1 && nuetral2 && nuetral3 && nuetral4)
    {
      nuetral= true;
    }      
  }
} 

//leaning left
void LeanLeft( void )
{

  boolean leaningLeft = false; //all actuators are in  position
  boolean leaningLeft1 = false; // checking actuator 1 in position
  boolean leaningLeft2 = false; // checking actuator 2 in position
  boolean leaningLeft3 = false; // checking actuator 3 in position
  boolean leaningLeft4 = false; // checking actuator 4 in position
//64 will stop the motor 1
//192 is stop for motor 2
//All MC's active to set a nuetral position  do i need to worry about this? 
// I'm setting the MC's with every loop check
  while (!leaningLeft)
  { 
    // Reading in values for actuator position, lower is more extended
    Actuator1Value = analogRead(Actuator1);
    Actuator2Value = analogRead(Actuator2);
    Actuator3Value = analogRead(Actuator3);
    Actuator4Value = analogRead(Actuator4);
    // checking and adjusting actuator 1
    if (Actuator1Value >= 1015)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator1Value <= 995)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator1Value > 995) && (Actuator1Value < 1015))
     leaningLeft1 = true;
    
    
      // checking and adjusting actuator 2
    if (Actuator2Value >= 55)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator2Value <= 35)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator2Value > 35) && (Actuator2Value < 55))
     leaningLeft2 = true;     
     
    
    // checking and adjusting actuator 3
    if (Actuator3Value >= 55)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator3Value <= 35)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator3Value > 35) && (Actuator3Value < 55))
     leaningLeft3= true;  
    
    
    // checking and adjusting actuator 4
    if (Actuator4Value >= 1015)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator4Value <= 995)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator4Value > 995) && (Actuator4Value < 1015))
     leaningLeft4 = true;       
    
    // checking to make sure all actuators are in right position before leaving loop.  
    if (leaningLeft1 && leaningLeft2 && leaningLeft3 && leaningLeft4)
    {
      leaningLeft= true;
    }      
  }

} 

// Leaning right
void LeanRight(void)
{

  boolean leaningRight  = false; //all actuators are in  position
  boolean leaningRight1 = false; // checking actuator 1 in position
  boolean leaningRight2 = false; // checking actuator 2 in position
  boolean leaningRight3 = false; // checking actuator 3 in position
  boolean leaningRight4 = false; // checking actuator 4 in position
//64 will stop the motor 1
//192 is stop for motor 2
// I'm setting the MC's with every loop check
  while (!leaningRight)
  { 

    // Reading in values for actuator position, lower is more extended
    Actuator1Value = analogRead(Actuator1);
    Actuator2Value = analogRead(Actuator2);
    Actuator3Value = analogRead(Actuator3);
    Actuator4Value = analogRead(Actuator4);    
    // checking and adjusting actuator 1
    if (Actuator1Value >= 55)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator1Value <= 35)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator1Value > 35) && (Actuator1Value < 55))
     leaningRight1 = true;
    
    
      // checking and adjusting actuator 2
    if (Actuator2Value >= 1015)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator2Value <= 995)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator2Value > 995) && (Actuator2Value < 1015))
     leaningRight2 = true;     
     
    
    // checking and adjusting actuator 3
    if (Actuator3Value >= 1015)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator3Value <= 995)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator3Value > 995) && (Actuator3Value < 1015))
     leaningRight3= true;  
    
    
    // checking and adjusting actuator 4
    if (Actuator4Value >= 55)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator4Value <= 35)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator4Value > 35) && (Actuator4Value < 55))
     leaningRight4 = true;       
    
    // checking to make sure all actuators are in right position before leaving loop.  
    if (leaningRight1 && leaningRight2 && leaningRight3 && leaningRight4)
    {
      leaningRight= true;
    }      
  }
}
//Leaning the upper body forward
void LeanForward(void)
{

  boolean leaningForward = false; //all actuators are in  position
  boolean leaningForward1 = false; // checking actuator 1 in position
  boolean leaningForward2 = false; // checking actuator 2 in position
  boolean leaningForward3 = false; // checking actuator 3 in position
  boolean leaningForward4 = false; // checking actuator 4 in position
//64 will stop the motor 1
//192 is stop for motor 2
// I'm setting the MC's with every loop check
  while (!leaningForward)
  { 
    // Reading in values for actuator position, lower is more extended
    Actuator1Value = analogRead(Actuator1);
    Actuator2Value = analogRead(Actuator2);
    Actuator3Value = analogRead(Actuator3);
    Actuator4Value = analogRead(Actuator4);    
    // checking and adjusting actuator 1
    if (Actuator1Value >= 55)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator1Value <= 35)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator1Value > 35) && (Actuator1Value < 55))
     leaningForward1 = true;
    
    
      // checking and adjusting actuator 2
    if (Actuator2Value >= 55)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator2Value <= 35)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator2Value > 35) && (Actuator2Value < 55))
     leaningForward2 = true;     
     
    
    // checking and adjusting actuator 3
    if (Actuator3Value >= 1015)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator3Value <= 995)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator3Value > 995) && (Actuator3Value < 1015))
     leaningForward3= true;  
    
    
    // checking and adjusting actuator 4
    if (Actuator4Value >= 1015)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator4Value <= 995)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator4Value > 995) && (Actuator4Value < 1015))
     leaningForward4 = true;       
    
    // checking to make sure all actuators are in right position before leaving loop.  
    if (leaningForward1 && leaningForward2 && leaningForward3 && leaningForward4)
    {
      leaningForward= true;
    }      
  }
}
//Leaning backwards
void LeanBackward(void)
{

  boolean leaningBackward = false; //all actuators are in  position
  boolean leaningBackward1 = false; // checking actuator 1 in position
  boolean leaningBackward2 = false; // checking actuator 2 in position
  boolean leaningBackward3 = false; // checking actuator 3 in position
  boolean leaningBackward4 = false; // checking actuator 4 in position
//64 will stop the motor 1
//192 is stop for motor 2
// I'm setting the MC's with every loop check
  while (!leaningBackward)
  { 
    // Reading in values for actuator position, lower is more extended
    Actuator1Value = analogRead(Actuator1);
    Actuator2Value = analogRead(Actuator2);
    Actuator3Value = analogRead(Actuator3);
    Actuator4Value = analogRead(Actuator4);
    // checking and adjusting actuator 1
    if (Actuator1Value >= 1015)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator1Value <= 995)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator1Value > 995) && (Actuator1Value < 1015))
     leaningBackward1 = true;
    
    
      // checking and adjusting actuator 2
    if (Actuator2Value >= 1015)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator2Value <= 995)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator2Value > 995) && (Actuator2Value < 1015))
     leaningBackward2 = true;     
     
    
    // checking and adjusting actuator 3
    if (Actuator3Value >= 55)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator3Value <= 35)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator3Value > 35) && (Actuator3Value < 55))
     leaningBackward3= true;  
    
    
    // checking and adjusting actuator 4
    if (Actuator4Value >= 55)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator4Value <= 35)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator4Value > 35) && (Actuator4Value < 55))
     leaningBackward4 = true;       
    
    // checking to make sure all actuators are in right position before leaving loop.  
    if (leaningBackward1 && leaningBackward2 && leaningBackward3 && leaningBackward4)
    {
      leaningBackward= true;
    }      
  }
}

//Move entire body up
void MoveUp(void)
{

  boolean movedUp = false; //all actuators are in  position
  boolean movedUp1 = false; // checking actuator 1 in position
  boolean movedUp2 = false; // checking actuator 2 in position
  boolean movedUp3 = false; // checking actuator 3 in position
  boolean movedUp4 = false; // checking actuator 4 in position
//64 will stop the motor 1
//192 is stop for motor 2
//All MC's active to set a nuetral position  do i need to worry about this? 
// I'm setting the MC's with every loop check
  while (!movedUp)
  { 
    // Reading in values for actuator position, lower is more extended
    Actuator1Value = analogRead(Actuator1);
    Actuator2Value = analogRead(Actuator2);
    Actuator3Value = analogRead(Actuator3);
    Actuator4Value = analogRead(Actuator4);    
    // checking and adjusting actuator 1
    if (Actuator1Value >= 55)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator1Value <= 35)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator1Value > 35) && (Actuator1Value < 55))
     movedUp1 = true;
    
    
      // checking and adjusting actuator 2
    if (Actuator2Value >= 55)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator2Value <= 35)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator2Value > 35) && (Actuator2Value < 55))
     movedUp2 = true;     
     
    
    // checking and adjusting actuator 3
    if (Actuator3Value >= 55)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator3Value <= 35)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator3Value > 35) && (Actuator3Value < 55))
     movedUp3= true;  
    
    
    // checking and adjusting actuator 4
    if (Actuator4Value >= 55)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator4Value <= 35)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator4Value > 35) && (Actuator4Value < 55))
     movedUp4 = true;       
    
    // checking to make sure all actuators are in right position before leaving loop.  
    if (movedUp1 && movedUp2 && movedUp3 && movedUp4)
    {
      movedUp= true;
    }      
  }
}

//moving all actuators to the down position
void MoveDown(void)
{

  boolean movedDown = false; //all actuators are in  position
  boolean movedDown1 = false; // checking actuator 1 in position
  boolean movedDown2 = false; // checking actuator 2 in position
  boolean movedDown3 = false; // checking actuator 3 in position
  boolean movedDown4 = false; // checking actuator 4 in position
//64 will stop the motor 1
//192 is stop for motor 2
//All MC's active to set a nuetral position  do i need to worry about this? 
// I'm setting the MC's with every loop check
  while (!movedDown)
  { 
    // Reading in values for actuator position, lower is more extended
    Actuator1Value = analogRead(Actuator1);
    Actuator2Value = analogRead(Actuator2);
    Actuator3Value = analogRead(Actuator3);
    Actuator4Value = analogRead(Actuator4);    
    // checking and adjusting actuator 1
    if (Actuator1Value >= 1015)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator1Value <= 995)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator1Value > 995) && (Actuator1Value < 1015))
     movedDown1 = true;
    
    
      // checking and adjusting actuator 2
    if (Actuator2Value >= 1015)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator2Value <= 995)
    {
      digitalWrite(MC1, HIGH);  
      digitalWrite(MC2, LOW); 
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator2Value > 995) && (Actuator2Value < 1015))
     movedDown2 = true;     
     
    
    // checking and adjusting actuator 3
    if (Actuator3Value >= 1015)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_FORWARD));    
    }
    if (Actuator3Value <= 995)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR1_FULL_REVERSE));
    }
    if ((Actuator3Value > 995) && (Actuator3Value < 1015))
     movedDown3= true;  
    
    
    // checking and adjusting actuator 4
    if (Actuator4Value >= 1015)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_FORWARD));    
    }
    if (Actuator4Value <= 995)
    {
      digitalWrite(MC1, LOW);  
      digitalWrite(MC2, HIGH);
      SaberSerial.write(byte(SABER_MOTOR2_FULL_REVERSE));
    }
    if ((Actuator4Value > 995) && (Actuator4Value < 1015))
     movedDown4 = true;       
    
    // checking to make sure all actuators are in right position before leaving loop.  
    if (movedDown1 && movedDown2 && movedDown3 && movedDown4)
    {
      movedDown= true;
    }      
  }
}

//this function will demo all the movements, notworking. 
//Isn't needed
void Demo(void)
{

 Nuetral();
 LeanLeft();
 LeanRight();
   
 Nuetral();
 LeanForward();
 LeanBackward();
 
 Nuetral();
 MoveUp();
 MoveDown();
}

void loop ( )
{
/*
  digitalWrite(MC1, LOW);  
  digitalWrite(MC2, LOW);

// for later in the project when it received command from a computer. 
  switch (command) 
  {
    case 'n':
      Nuetral();
      break; 
    case 'l':
      LeanLeft();
      break;
    case 'r':
      LeanRight();
      break;
    case 'u':
      MoveUp();
      break;
    case 'd':
      MoveDown();
      break;
    case 'f':
      LeanFoward();
      break;
    case 'b'
      LeanBackward();
      break;  
    case 'd':
      Demo();
      break;  
    
  }*/ 
  
  
  


//64 will stop the motor 1
//192 is stop for motor 2
//All MC's active to set a nuetral position  
    digitalWrite(MC1, LOW);  
    digitalWrite(MC2, LOW);
   // there appears to be a bug where it will only do two functions in a row. All functions work individualy
  Nuetral();
  LeanLeft();
  LeanRight();
  Nuetral();
  LeanForward();
  LeanBackward();
  Nuetral();
  MoveUp();
  MoveDown();
//   Demo(); //demo of all the movements, not really needed since i'm doing that already with my main loop. 

}
