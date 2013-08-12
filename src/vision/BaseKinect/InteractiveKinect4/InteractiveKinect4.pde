/* =====================================
   Rami Alshafi
   ECE 579: INTELLIGENT ROBOTICS II
   Portland State University
   Dr.Perkowski
   Final Version as of Winter 2013 
   ====================================*/
// ==== Libraries Used ===========//
import processing.serial.*; // Serial Library (http://processing.org/reference/libraries/serial/index.html)
import processing.net.*;
import processing.video.*;

import javax.imageio.*;
import java.awt.image.*; 
import java.net.*;
import java.io.*;

import ddf.minim.*; // Audio library (http://code.compartmental.net/tools/minim/)
import SimpleOpenNI.*; // OpenNI library for Processing (https://code.google.com/p/simple-openni/)

//=========== Object declarations======//
SimpleOpenNI kinect; // Kinect object to use OpenNI
Serial port; // Serial port object to use serial communication library
Minim minim; // minim object to use minim library
AudioPlayer player; // player instant of the AudioPlayer class of Minim
Client client;   // client object for server communication

//========== variables declaration ======//
PVector handVector = new PVector(); // vector for hand real world fresh XYZ coordinates 
PVector mapHandVector = new PVector(); // vector for hand projective XYZ coordinates
boolean DisBF = true; // Distance Satisfaction Flag, set to True at start to give priority for Rotation satisfaction first
boolean roto  = false; // rotation satisfaction Flag, set to False at start to give priority for Rotation satisfaction first
int close; // integer variable to host the closest point
int closeX; // the x position of the closest point
int closeY; // the y position of the closest point
int base_x = 540;
//int LEFT = 108, RIGHT = 106, STOP= 107, FORWARD = 105, BACKWARD = 44; // integer varialbles to host commands protocol with Arduino
int LEFT = 97, RIGHT = 100, STOP= 120, FORWARD = 119, BACKWARD = 115, STRAFE_LEFT = 122, STRAFE_RIGHT = 99; // integer varialbles to host commands protocol with Arduino
//int BoxX = 740, BoxW = 50,  HlineX1 = 665, HlineX2= 815, VlineX1 = 740, VlineX2 = 740;  // Horizontal variables for robot base animation
int BoxX = base_x, BoxW = 50,  HlineX1 = 465, HlineX2= 615, VlineX1 = base_x, VlineX2 = base_x;  // Horizontal variables for robot base animation
int BoxY = 400, BoxH = 50, HlineY1 = 400, HlineY2 = 400, VlineY1 = 325, VlineY2 = 475;  // vertical variables for robot base animation
int bowStroke = 15;
int send = 4; // integer variable to host the outgoing command to serial
boolean handsTrackFlag = false; // Hand tracking flag, set to false at start since we are not tracking the hand at first



int savedTime;
int totalTime = 5000;
int wait_thinking = 0;
int idle_action_duration = 0;
int idle_action=0;
boolean idle_wait_done = false;
float dir=0;

int buttonTimeout = 0;

//============== Client variables ========
String messageFromServer = "";
String previousType = "";
String previousCmd = "";
String cmdToDisplay = "";
boolean kinect_flag = false;
boolean speech_flag = false;
boolean tablet_flag = false;

//============== Text positions ===========
/*int status_x = 740;
int status_y = 20;
int instruction_x = 740;
int instruction_y = 180;
int idle_x = 740;
int idle_y = 50;
int command_x = 740;
int command_y = 400;
int linespacing = 30;
int textsize1 = 15;
int textsize2 = 20;*/


int status_x = base_x;
int status_y = 20;
int instruction_x = base_x;
int instruction_y = 180;
int idle_x = base_x;
int idle_y = 50;
int command_x = base_x;
int command_y = 350;
int linespacing = 30;
int textsize1 = 15;
int textsize2 = 20;

int button_y = 50;
int playButton_x = 100;
int playButton_y = button_y;
int followButton_x = 280;
int followButton_y = button_y;
int rgbButton_x = 360;
int rgbButton_y = 120;
int buttonWidth = 150;
int buttonHeight = 40;
int buttonWidthOffset = buttonWidth/2;
int buttonHeightOffset = buttonHeight/2;

String song = "Play Music"; // String variable hosting the music button display, set to play music at start
String followhand = "Follow Hand";
String rgbmode = "Show Depth";

boolean inFollowHandBox = false;
boolean followHandFlag = false; // Move base to follow hand if true
boolean inPlayMusicBox = false;
boolean playMusicFlag = false;
boolean inRgbBox = false;
boolean rgbFlag = false;

//============== Screen streaming variables =====
// This is the port we are sending to
int clientPort = 9100; 
// This is our object that sends UDP out
DatagramSocket ds;
PImage screen;
SenderThread sender;
//============== setup function =========//
void setup()
{
  String portName = "/dev/ttyACM3";
  port = new Serial(this, portName, 9600); // initialize the serial object, selected port and buad rate
  kinect = new SimpleOpenNI(this, SimpleOpenNI.RUN_MODE_MULTI_THREADED); // initialize the kinect object
  kinect.setMirror(true); // Mirror the depth image
  kinect.enableDepth(); // enable the depth camera of the kinect
  kinect.enableGesture(); // enable the Gesture class of the Kinect
  kinect.enableHands(); // enable the hands class of the kienct
  kinect.enableRGB();
  kinect.addGesture("RaiseHand"); // add the RaiseHand Gesture
  minim = new Minim(this); // initialize the Minim object
  //size (kinect.depthWidth()+200, kinect.depthHeight()+100); // display the depth image and extra space for User Interface // ORIGINAL VALUES
  size (kinect.depthWidth(), kinect.depthHeight()); // display the depth image and extra space for User Interface
    //String portName = Serial.list()[6]; // Select the Serial port number CUSTIMIZE!!!!!!!!!!!!! this is hardware specific 
  //String portName = "/dev/ttyACM0";
  //port = new Serial(this, portName, 9600); // initialize the serial object, selected port and buad rate
  player = minim.loadFile("/home/mcecsbot/Music/i wanna love ya.mp3"); // Load the music file, MUST BE IN THE SKETCH FOLDER to be loaded!!
  savedTime = millis();  //start internal timer, counts in milliseconds
  
  client = new Client(this, "127.0.0.1", 8008);
  client.write("iam:kinect");
  
  sender = new SenderThread(kinect.depthWidth(), kinect.depthHeight());
  sender.start(); 
}
//======================== Main Function=============//
void draw()
{
  int passedTime;
  background(0); // clean the background with black color
  close = 6000; // set the closest point to 6000 mm as a starting point 
  kinect.update(); // update the kinect
  if (rgbFlag) {
    screen = kinect.depthImage();
  } else {
    screen = kinect.rgbImage();
  }
  
  //sendScreen(screen);
  //image(kinect.depthImage(), 0, 0); // display the depth image starting from X=0, Y=100
  image(screen,0,0);
  int [] depthValues = kinect.depthMap();
  
  //int [] depthValues = kinect.depthMap();// Get pixels depth values
  //updateScreen();
  //screen = get(0,0,width,height);
  //sender.setImage(screen);
  //println(width+"-"+height);
  
  rectMode(CORNER);
  noStroke();
  fill(0,0,0,127);
  rect(440, 0, 200, 480);
  
  //=========== Check message from server ===========
  if (client.available() > 0) {
    messageFromServer = client.readString();
    parseMessage(messageFromServer);
  }
  
  //=========== Finding the Closest point alnog with its X-Y coordinates ===========//
  for(int y = 0; y<480; y++){ // go over the columns
    for(int x=0; x<640;x++){ // go over the rows
      int i = x + y*640; // pixel ID
      int currentDis = depthValues[i]; // distance for this pixel
      if(currentDis > 0 && currentDis < close){ // Check if this distance is the closest so far?
        close = currentDis;// If so, record it
        closeX = x; // get its X coordinate
        closeY = y; // get its Y coordinate
      }//otherwise, do not record it
    }
  } 
// ======== End of Finding the closest point ===========//
 if(close < 600 && close > 0){// check if the closest point so far is too close
   if (send != STOP){ // If so, then check if we have already sent this command
   send = STOP;// if not, set the send variable to STOP
   port.write(send); // send it   
   println("STOP, obstacle  "+send); // print the sent value to the console for checking, (unnecessary but useful for debuging)
   client.write(formatMessage("base", "stop"));
    if (previousCmd != "tooclose") {
        client.write(formatMessage("status","tooclose"));
        previousCmd = "tooclose";
      }   
   }
   
   // otherwise, we do not need to re-send the command. just display warning
   //============Warning Display======================//
   rectMode(CENTER);
   fill(255,255,0);
   rect(closeX, closeY, 125, 40,15);
   textAlign(CENTER);
   textSize(20);
   fill(255, 0, 0);
   textAlign(CENTER, CENTER);
   text("OBSTACLE",closeX, closeY);
   /*rectMode(CORNER);
   fill(255, 255, 0);
   rect(4, 40, 600, 40);
   textAlign(CENTER, CENTER);
   fill(255, 0, 0);*/
   rectMode(CENTER);
   fill(255,255,0);
   rect(status_x, status_y, 200, 158);
   textAlign(CENTER,CENTER);
   fill(255,0,0);
   textSize(24);
   text("OBSTACLE\nDETECTED", status_x, status_y+30);
   //text("Obstacle on the way, CANNOT follow you anymore!!!", 300, 50);
   //writeInstructionStatus("Obstacle",1);
   writeInstructionStatus("Something is\ntoo close.\nI won't move\nuntil the path\nis cleared.",0);
   rectMode(CENTER);
   fill(255, 0, 0);
   stroke(255,255, 0);
   strokeWeight(10);
   rect(BoxX, BoxY, BoxW, BoxH,15);
   line(HlineX1, HlineY1, HlineX2, HlineY2);
   line(VlineX1, VlineY1, VlineX2, VlineY2);
   strokeWeight(1);
   
   //========== end of warning display ====================//
 } else if(handsTrackFlag == true) {  // Check if we are tracking the hand?
   
   if (previousCmd != "trackhand") {
     client.write(formatMessage("all","kinect_flag:true"));
     client.write(formatMessage("status","trackhand"));
     client.write(formatMessage("arm","wave"));
     previousCmd = "trackhand";
   }
   if (!followHandFlag && !rgbFlag) {
     writeInstructionStatus("A hand has been\n detected.\nHover hand over\nbuttons Into interact.",0);
     writeInstructionStatus("Tracking Hand", 1);
   }
   updateScreen();
   // if so, get its information
   savedTime = millis();                // reset millis time
   kinect.convertRealWorldToProjective(handVector, mapHandVector); // convert hand position coordinates to projective
   ellipse(mapHandVector.x, mapHandVector.y, 50, 50); // draw a circle on the hand position

   //========== get Hand Distance ============//
   int handDistance = int(mapHandVector.x) + (int(mapHandVector.y)*640); // use the hands X-Y coordinates to select a pixel
   int millimeters = depthValues[handDistance]; // read the depth value of that pixel on which is the hand
   //========== end of getting hand distance =========//
   textSize(20);
   fill(128, 128, 220);
   textAlign(CENTER, CENTER);
   // a choice to display the hand position and distance instead of a circle
   text(millimeters+"\n"+"("+int(mapHandVector.x)+" , "+int(mapHandVector.y)+")", mapHandVector.x, mapHandVector.y-30);
   // ============= hand has been detected, inform the user (UI) ==========//
            
    //writeInstructionStatus("Session is ON \n The Robot \n should be \n following \n your hand\nNOW",0);
    if (followHandFlag) {
      rectMode(CENTER);
      fill(255);
      stroke(255);
      rect(BoxX, BoxY, BoxW, BoxH);
      line(HlineX1, HlineY1, HlineX2, HlineY2);
      line(VlineX1, VlineY1, VlineX2, VlineY2);
    }
    /*stroke(200,200,200);
    fill(100,100,200,0.5);
    strokeWeight(3);
    rect(300, 125, 130, 40, 15);
    rectMode(CENTER);            
    textAlign(CENTER, CENTER);
    fill(255);
    text(song, 300, 125);*/
    makeCommandBox(song, playButton_x, playButton_y, buttonWidth, buttonHeight);
    makeCommandBox(followhand, followButton_x, followButton_y, buttonWidth, buttonHeight);
    makeCommandBox(rgbmode, rgbButton_x, rgbButton_y, buttonWidth, buttonHeight);
        
//============== end of display ========================//
    //if (mapHandVector.x-30 > 100 && mapHandVector.x-30 < 230 && mapHandVector.y > 100 && mapHandVector.y < 150 && !inPlayMusicBox){
    //must substract by buttonWidth/HeightOffset because the button_x/y are in center
    if (mapHandVector.x > playButton_x-buttonWidthOffset && mapHandVector.x < playButton_x+buttonWidthOffset && mapHandVector.y > playButton_y-buttonHeightOffset && mapHandVector.y < playButton_y+buttonHeightOffset && !inPlayMusicBox){
      // check if the hand is on the music button we created in the display?
      if(!player.isPlaying() && song.equals("Play Music") && !playMusicFlag){// if so, check if the music is not playing
         player.play();// if so, play music
         song = "Pause Music";// replace music button text with "Pause" instaed of "Play"
         playMusicFlag = true;

       }else if (player.isPlaying() && playMusicFlag){// otherwise, check if the music is playing
         player.pause();// if so, pause the music
         song = "Play Music";// replace music button text with "Play" instaed of "pause"
         playMusicFlag = false;

       }//otherwise, resume execution
       inPlayMusicBox = true;
       
    }// otherwise, resume executon
    
    if (mapHandVector.x < playButton_x-buttonWidthOffset || mapHandVector.x > playButton_x+buttonWidthOffset || mapHandVector.y < playButton_y-buttonHeightOffset || mapHandVector.y > playButton_y+buttonHeightOffset) {
      inPlayMusicBox = false; 
    }
      
    //if (mapHandVector.x-30 > 300 && mapHandVector.x-30 < 430 && mapHandVector.y > 100 && mapHandVector.y < 150 && !inFollowHandBox){
    if (mapHandVector.x-30 > followButton_x-buttonWidthOffset && mapHandVector.x-30 < followButton_x+buttonWidthOffset && mapHandVector.y > followButton_y-buttonHeightOffset && mapHandVector.y < followButton_y+buttonHeightOffset && !inFollowHandBox){
      // check if the hand is on the music button we created in the display?
      buttonTimeout = millis();
      if (followhand.equals("Follow Hand") && !followHandFlag) {
        followhand = "Stop Following";
        followHandFlag = true;
        
       
      } else if (followhand.equals("Stop Following")) {
        followhand = "Follow Hand";
        followHandFlag = false;
        //send = STOP;// if not, set the send variables to STOP to be sent
        port.write(STOP);// send it        
      }
      inFollowHandBox = true;      
    }
    
    if (mapHandVector.x-30 < followButton_x-buttonWidthOffset || mapHandVector.x-30 > followButton_x+buttonWidthOffset || mapHandVector.y < followButton_y-buttonHeightOffset || mapHandVector.y > followButton_y+buttonHeightOffset) {      
      inFollowHandBox = false;      
    }
    
    if (mapHandVector.x-30 > rgbButton_x-buttonWidthOffset && mapHandVector.x-30 < rgbButton_x+buttonWidthOffset && mapHandVector.y > rgbButton_y-buttonHeightOffset && mapHandVector.y < rgbButton_y+buttonHeightOffset && !inRgbBox){
      // check if the hand is on the music button we created in the display?
      buttonTimeout = millis();
      if (rgbmode.equals("Show Depth") && !rgbFlag) {
        rgbmode = "Show RGB";
        rgbFlag = true;
        
       
      } else if (rgbmode.equals("Show RGB")) {
        rgbmode = "Show Depth";
        rgbFlag = false;
        //send = STOP;// if not, set the send variables to STOP to be sent
        port.write(STOP);// send it        
      }
      inRgbBox = true;      
    }
    
    if (mapHandVector.x-30 < rgbButton_x-buttonWidthOffset || mapHandVector.x-30 > rgbButton_x+buttonWidthOffset || mapHandVector.y < rgbButton_y-buttonHeightOffset || mapHandVector.y > rgbButton_y+buttonHeightOffset) {      
      inRgbBox = false;      
    }
    
    if (followHandFlag) {
      writeInstructionStatus("Following Hand...",1);
      writeInstructionStatus("Move hand around\nto lead the robot.",0);
    } else if (rgbFlag) {
      writeInstructionStatus("RGB Mode",1);
      writeInstructionStatus("Now showing\nfull color image.",0);
    }
    updateScreen();
    //println(inFollowHandBox);
            
    if(roto == false) {// check if the rotation satisfaction is not satisfied
      if (mapHandVector.x < 0.25*kinect.depthWidth() && followHandFlag) { // if so, check if the hand is in the left
        if ((send != RIGHT) && (send != STRAFE_RIGHT)) {// if so, check if we have already sent this command
          float r = random(10);
          String cmd = "";
          if (r > 5) {
            send = RIGHT;// if not, then set the send variable to left to be sent
            cmd = "right";
          } else {
            send = STRAFE_RIGHT;
            cmd = "straferight";
          }
         
          port.write(send);// send it
          println("Right  "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base",cmd)); 
          //======== display the command =======//
          
          //println("rotofalse, right");
          writeCommand("Right",0); 
    
        } else {// otherwise, we do not need to re-send the command, just display to the user to inform 
        //=========== LEFT display======================//
          //println("rotofalse, left");
          
          writeCommand("Left",0);
          
          //text("Rotation satisfied? "+roto,340, 20);
          rectMode(CENTER);
          fill(255, 136, 0);
          stroke(255);
          rect(BoxX, BoxY, BoxW, BoxH);
          line(HlineX1, HlineY1, HlineX2, HlineY2);
          line(VlineX1, VlineY1, VlineX2, VlineY2);
          stroke(255, 0, 0);
          strokeWeight(10);
          line(BoxX, BoxY, HlineX1, HlineY1);
          strokeWeight(1);
          //======= end of LEFT display =========//
          
        }
        //updateScreen();
      } else if (mapHandVector.x > 0.75*kinect.depthWidth() && followHandFlag) { // otherwise, check if hand is inthre right side
        if ((send != LEFT) && (send != STRAFE_LEFT)) {// if so, check if we have already sent this command
          float r = random(10);
          String cmd = "";
          if (r > 5) {
            send = LEFT;// if not, set the sne dvariable to right to be sent
            cmd = "left";
          } else {
            send = STRAFE_LEFT;
            cmd = "strafeleft";
          }
         
          port.write(send);//send it
          println("Left "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base",cmd));
          //===== display the command ======//
          writeCommand("Left", 0);

        } else {//otherwise, we do not need to re-send the command
        //============ RIGHT display==============//
            /*textSize(20);
            fill(255, 0, 0);
            textAlign(CENTER, CENTER);
            text("Right", command_x, command_y);*/
            writeCommand("Right",0);
            //text("Rotation satisfied? "+roto,340, 20);
            rectMode(CENTER);
            fill(255, 136, 0);
            stroke(255);
            rect(BoxX, BoxY, BoxW, BoxH);
            line(HlineX1, HlineY1, HlineX2, HlineY2);
            line(VlineX1, VlineY1, VlineX2, VlineY2);
            stroke(255, 0, 0);
            strokeWeight(bowStroke);
            line(BoxX, BoxY, HlineX2, HlineY2);
            strokeWeight(1);            
            //======= end of RIGHT display==========//

        }
        //updateScreen();
      } else if (mapHandVector.x <= 0.75*kinect.depthWidth() && mapHandVector.x >= 0.25*kinect.depthWidth() && followHandFlag) { 
        // otherwise, check if the hand is in the middle
                roto = true;// if so, set the rotation flag to true, so it is not executed again before executing the Distance satisfation
                DisBF = false;// also, set the distance satisfaction flag to false so it can be executed next
                fill(0,255, 0);
                textAlign(CENTER, CENTER);
                //text("Rotation satisfied? "+roto,340, 20);
              if (send != STOP) {// check if we have already sent this command 
                send = STOP;// if not, set the send variables to STOP to be sent
                port.write(send);// send it
                println("stay "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
                client.write(formatMessage("base","stop"));
                //==== display the command====//
               
              } else {// otherwise, we do not need to re-send the command, just inform the user
              //================= STOP display=================//
                
                rectMode(CENTER);
                fill(0, 255, 0);
                stroke(255);
                rect(BoxX, BoxY, BoxW, BoxH);
                line(HlineX1, HlineY1, HlineX2, HlineY2);
                line(VlineX1, VlineY1, VlineX2, VlineY2);
              }// end of STOP display
              writeCommand("Stay",0);
              //updateScreen();

      }// otherwise, this is unharmful error, resume execution
    } else if(DisBF == false && followHandFlag) {// otherwise, check if the distance satisfaction flag is false
      
      if (millimeters < 900 && millimeters > 0) {// if so, check if the hand is close. execlude the 0! its NOISE!!
        if (send != BACKWARD){// if so, check if we have already 
          send = BACKWARD;// if not, set the send variable to move BACKWARD to be sent
          port.write(send);// send it
          println("BACKWARD "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base","reverse"));
          //====== display the backaward command============//
          
        } else{// otherwise, we do not need to re-send the back ward command again
        //====================== BACKWARD display=====================//
          
          //text("Distance satisfied? "+DisBF,110,20);
          rectMode(CENTER);
          fill(255, 136, 0);
          stroke(255);
          rect(BoxX, BoxY, BoxW, BoxH);
          line(HlineX1, HlineY1, HlineX2, HlineY2);
          line(VlineX1, VlineY1, VlineX2, VlineY2);
          stroke(255, 0, 0);
          strokeWeight(bowStroke);
          line(BoxX, BoxY, VlineX2, VlineY2);
          strokeWeight(1);
          //=============== end of BACKWARD display ====================//
          
        }
        writeCommand("Backward",0);
        //updateScreen();

      } else if (millimeters > 1200 && millimeters > 0) {//otherwise, check if the hand is far. execlude the 0! it is NOISE!!!
        if(send != FORWARD){// chekc if we have already sent this command
          send= FORWARD;// if not, set the send variable to move forward to be sent
          port.write(send);// send it
          println("FORWARD "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base","forward"));
          //======= display the FORWARD command============//
          
        }else{//otherwise, we do not need to resend the FORWARD command again
        //============== just display the FORWARD command ======================//
          
          //text("Distance satisfied? "+DisBF,110,20);
          rectMode(CENTER);
          fill(255, 136, 0);
          stroke(255);
          rect(BoxX, BoxY, BoxW, BoxH);
          line(HlineX1, HlineY1, HlineX2, HlineY2);
          line(VlineX1, VlineY1, VlineX2, VlineY2);
          stroke(255, 0, 0);
          strokeWeight(bowStroke);
          line(BoxX, BoxY, VlineX1, VlineY1);
          strokeWeight(1);
          //=========== end of FORWARD command =====================//
        }
          writeCommand("Forward",0);
          //updateScreen();

      } else if (millimeters < 1200 && millimeters > 900) {// check if the hand is in the middle
        DisBF = true;// if so, set the distance satisfaction flag to true, so it does not get executed again before the rotation satisfaction
        roto = false;// also, set the rotation satisfaction flag to false so it get executed next
        //textSize(20);
        //fill(0, 255, 0);
        //textAlign(CENTER, CENTER);
        //text("Distance satisfied? "+DisBF,110,20);
        if (send != STOP){// check if we have already sent this command 
          send = STOP;// if not, set the send variable to STOP to be sent
          port.write(send);// send it
          println("stay "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base","stop"));
              
        }else{//otherwise, we do not need to re-send the STOP command again
        //============= Display the STOP command ========================//
        
          rectMode(CENTER);
          fill(0, 255, 0);
          stroke(255);
          rect(BoxX, BoxY, BoxW, BoxH);
          line(HlineX1, HlineY1, HlineX2, HlineY2);
          line(VlineX1, VlineY1, VlineX2, VlineY2);
              //============= end of STOP display =====================//
        }
         writeCommand("Stay", 0);
         //updateScreen();

      }// other wise, this is unharmful error, resume execution
    }// otherwise, distance satisfied, resume execution
    
  } else if (kinect_flag || speech_flag) {
   
      /*textSize(20);
      fill(255);
      textAlign(CENTER, CENTER);
      text("Interaction mode",status_x,status_y);*/
      writeInstructionStatus("Speech mode",1);
      if (speech_flag) {
        text("Speak, mortal.",status_x,status_y+30);
        if (!cmdToDisplay.equals("")) writeCommand(cmdToDisplay,0);
      } else if (kinect_flag) {
        text("Gesture!", status_x,status_y+30);
      }
      //displayDirectionIndicator();
      //updateScreen();
   
  } else { // otherwise, the hand is not being tracked. could be the begining of session, or hand is lost. Display instruction to detect hand  
    
    if (previousCmd != "idle") {
      println("Idle mode...");
      client.write(formatMessage("base","idle"));
      
      kinect_flag = false;  // set kinect_flag to false
      client.write(formatMessage("all","kinect_flag:false")); // when kinect_flag set to false, tell all the clients
      
      previousCmd = "idle";
      wait_thinking = int(random(3000, 5000));
      idle_action_duration = int(random(5000, 10000));
      println("wait = " + int(wait_thinking));
      savedTime = millis();
      idle_action = int(random(2));    // Randomly choose an idle action: 0 = roam, 1 = look for people      
      println("Pick: " + idle_action);
      
      if (idle_action == 1) {
        dir = random(2);
      }  
      idle_wait_done = false;
      
      // First thing to do is to tell the robot to stop
      if (send != STOP){// check if we have already sent this command 
          send = STOP;// if not, set the send variable to STOP to be sent
          port.write(send);// send it
          println("stay "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base","stop"));
      }
    }
    
    passedTime = millis() - savedTime;
    writeInstructionStatus("Idle", 1);
    
    if ((passedTime < int(wait_thinking)) && !idle_wait_done) {
      
      writeCommand("Hmm...what to do...",1);
      text("Passed time: " + passedTime/1000 + "s", idle_x, idle_y+linespacing);
      //updateScreen();
      
    } else {
      if (!idle_wait_done) {
        savedTime = millis();  // Record time after "thinking" phase as reference for duration of idle action
        idle_wait_done = true;  
      }          
    }
    println("kinect_flag: "+kinect_flag);
    println("speech_flag: "+speech_flag);
    
    //if (idle_wait_done) { // NOTE*** ADD CHECKS; only execute if all interaction flags are false: kinect_flag, speech_flag, and tablet_flag
    if (idle_wait_done && !kinect_flag && !speech_flag) {
      fill(0,255,0);
      if (idle_action==0) {
        text("Roaming ...",idle_x,idle_y);
        roaming(savedTime, idle_action_duration);
        //updateScreen();
      } else {
        text("Searching ...",idle_x-10,idle_y);
        searching(savedTime, idle_action_duration, dir);
        //updateScreen();
      }
    } 
    
    writeInstructionStatus("Raise a hand\nto start\ninteraction", 0);    
    //displayDirectionIndicator();
    
    //screen = get(0,0,width,height);
    //sender.setImage(screen);
    
    /*rectMode(CENTER);
    fill(0);
    stroke(255);
    rect(BoxX, BoxY, BoxW, BoxH);
    line(HlineX1, HlineY1, HlineX2, HlineY2);
    line(VlineX1, VlineY1, VlineX2, VlineY2);*/
    //========= end if instructions display================//
     if (player.isPlaying()){// check if the music is playing
             player.rewind();// if so, rewind it, nobody is here :/
     }//otherwise, music not playing, just resume execution
     //updateScreen();
  }
   int currenttime = millis();
   while (millis() - currenttime < 10) {}
  updateScreen();
  
}// return to the begining (End of draw loop)

void updateScreen() {
  //screen = get(0,0,width,height);
  //sender.setImage(screen);
  sender.updateImage();
}

void sendScreen(PImage image){
  sender.setImage(image); 
}

void writeInstructionStatus(String instruction, int type) {
  textSize(20);
  fill(255);  // white
  textAlign(CENTER, CENTER);
  if (type==0) {
    text(instruction, instruction_x, instruction_y);
  } else if (type==1) {
    text(instruction, status_x, status_y);
  }    
  //text(instruction, x, y);  
}

void writeCommand(String command, int type) {
  textSize(20);
  textAlign(CENTER, CENTER);
  if (type==0) {
    fill(255,0,0);  // green
    text(command, command_x, command_y);
  } else if (type==1) {
    fill(0,255,0);  // green
    text(command, idle_x, idle_y);
  }
}

void makeCommandBox(String command, int x, int y, int boxwidth, int boxheight) {
  stroke(200,200,200);
  
  strokeWeight(3);
  rectMode(CENTER);
  fill(100,100,200,100);
  rect(x, y, boxwidth, boxheight, 15);
              
  textAlign(CENTER, CENTER);
  fill(220,220,255);
  text(command, x, y);
}

void displayDirectionIndicator() {
  rectMode(CENTER);
  fill(0);
  stroke(255);
  rect(BoxX, BoxY, BoxW, BoxH);
  line(HlineX1, HlineY1, HlineX2, HlineY2);
  line(VlineX1, VlineY1, VlineX2, VlineY2);
}

void roam(int passedTime) {
    if (passedTime > 5000 && passedTime <10000){  // after a five second delay, go forward for five seconds
      if (send != FORWARD){
        send=FORWARD;
        port.write(send);//send command to go forward. Wall following and sensor based code to be added here
        println("Forward "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
      }
    }
    if(passedTime > 10000  && passedTime <15000){  // go RIGHT for five seconds 
      if (send != RIGHT){
        send=RIGHT;
        port.write(send);//send command to go RIGHT. Wall following and sensor based code to be added here
        println("Right "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
      }
    }
    if (passedTime > 15000  && passedTime <20000){  // go backward for five seconds
      if (send != BACKWARD){
        send=BACKWARD;
        port.write(send);//send command to go BACKWARD. Wall following and sensor based code to be added here
        println("Backward "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
      }
    }
    if(passedTime > 20000  && passedTime <25000){    // go right for five seconds
      if (send != LEFT){
        send=LEFT;
        port.write(send);//send command to go forward. Wall following and sensor based code to be added here
        println("Left "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
      }
    } 
}

void roaming(int savedTime, int duration) {
  int passed = millis() - savedTime;
  int remaining = (duration - passed)/1000;
  text("Remaining time: " + remaining, idle_x,idle_y+linespacing); 
  if (passed < duration) {
    if (send != FORWARD) {
      send=FORWARD;
      port.write(send);//send command to go forward. Wall following and sensor based code to be added here
      println("Forward "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
    }
  } else {
    idle_wait_done = false;
    previousCmd = "forward";
    client.write(formatMessage("base","idle"));
    client.write(formatMessage("all","kinect_flag:false"));
  }
      
}
void searching(int savedTime, int duration, float dir) {
  int passed = millis() - savedTime;
  int remaining = (duration - passed)/1000;
  int offset = 72;
  
  text("Remaining time: " + remaining, idle_x,idle_y+linespacing);
  if (passed < duration) {
    if (dir < 0.5) {
      text("right", idle_x+offset, idle_y);
      if (send != RIGHT) {
        send=RIGHT;        
        port.write(send);//send command to go forward. Wall following and sensor based code to be added here
        println("Right "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
      }
    } else {
      text("left", idle_x+offset, idle_y);
      if (send != LEFT) {
        send=LEFT;        
        port.write(send);//send command to go forward. Wall following and sensor based code to be added here
        println("Left "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
      }
    }
  } else {
    idle_wait_done = false;
    previousCmd = "searching";
    client.write(formatMessage("base","idle"));
    client.write(formatMessage("all","kinect_flag:false"));
  }
}


void onRecognizeGesture(String strGesture, PVector idPosition, PVector endPosition)
{// OpenNI callback function when hand gesture is recognized
  kinect.removeGesture(strGesture);// remove the gesture, we do not want to look for it again
  kinect.startTrackingHands(endPosition);// get the position of the hand
}

void onCreateHands(int handId, PVector pos, float time)
{// OpenNI callback function, it gets called after the gesture recognition function
  handsTrackFlag = true;// set the hand tracking flag to true
  handVector = pos;// store the hand position to vector variable
}

void onUpdateHands(int handId, PVector pos, float time)
{// OpenNI callback function to keep updating the hand position
  handVector = pos;//store updated hand position to vector variable
}

void onDestroyHands(int handId,float time)
{// OpenNI callback function. it gets called when hand is lost or not available
  handsTrackFlag = false;// set hand tracking flag to false
        if (send != STOP){// check if we have already sent this command 
          send = STOP;// if not, set the send variable to STOP to be sent
          port.write(send);// send it
          println("stay "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          textSize(20);
          fill(0,255,0);
          textAlign(CENTER, CENTER);
          text("Stay", command_x, command_y);
        }
  kinect.addGesture("RaiseHand");// add the gesture back because we deleted it already
}

//========== closing function===========//
// this is not needed but its purpose here is to close the program peacefuly.
// right now is set to be executed when the X button onthe keyboard gets pressed.
// I wish for this function to be executed when a wave gesture is detected
void keyPressed(){
  if (key == 'x') { 
  exit();
  }
}

//============= Parse message from server =========
void parseMessage(String msg) {
  String [] message = split(trim(msg),':');
  String sender, target, command;
  int msg_length = message.length;
  if (msg_length >= 3 && message[1].equals("base") && !message[0].equals("kinect")) {
    
    sender = trim(message[0]);
    target = trim(message[1]);
    command = trim(message[2]);
    
    /*if (msg_length > 3) {
      String [] command_args = {};
      for (int i = 3; i < msg_length; i++) {
        command_args[i-3] = message[i];
      }
    }*/
    
    String client_msg = "msg:OK. I received a command " +command.toUpperCase() + " from " + message[0].toUpperCase();
    //client.write(client_msg);
    println(client_msg);
    if (speech_flag) cmdToDisplay = command;
    // DEBUGGING STUFF
    /*
    //println(message.length);
    //println(target);
    //println(command);
    for (int i = 0; i<message.length; i++) {
      println(message[i]);      
    }
    */
    // END OF DEBUGGING STUFF 
    print("Command to execute: ");
    if (command.equals("stop")) {
      port.write(STOP);
      println("STOP");
    } else if (command.equals("forward")) {
      port.write(FORWARD);
      println("FORWARD");
    } else if (command.equals("right")) {
      port.write(RIGHT);
      println("RIGHT");
    } else if (command.equals("left")) {
      port.write(LEFT);
      println("LEFT");
    } else if (command.equals("reverse")) {
      port.write(BACKWARD);
      println("REVERSE"); 
    } else if (command.equals("strafeleft")) {
      port.write(STRAFE_LEFT);
      println("STRAFE_LEFT");
    } else if (command.equals("straferight")) {
      port.write(STRAFE_RIGHT);
      println("STRAFE_RIGHT");      
    } else { // Default position
      //port.write(STOP);
      println("Unknown command. So I'm stopping.");
    }
  } else if (msg_length == 4 && message[1].equals("all")) {
      // Broadcasted message, all clients must pay attention to this
      String flag = trim(message[2]);
      String value = trim(message[3]);
      println("Flag = " + flag);
      if (flag.equals("speech_flag")) {
        if (value.equals("true")) {
          speech_flag = true;          
        } else {
          speech_flag = false;
          cmdToDisplay=""; // clear variable
        }
        
      } else if (flag.equals("tablet_flag")) {
        if (value.equals("true")) {
          tablet_flag = true;
        } else {
          tablet_flag = false;
        }
      } else if (flag.equals("kinect_flag")) {
        if (value.equals("true")) {
          kinect_flag = true;
        } else {
          kinect_flag = false;
        }
      } else {
        println(message[2] + message[3]);
      }
      println(flag + ": " + value);
  } else {
    println("Message length is: " + message.length);
    //println(msg);
      if (msg_length > 1) {
      target = trim(message[0]);
      command = trim(message[1]);
      println(target);
      println("-------------");
      println(command);
      println("+++++++++++++");
      //println(message[2]);
      //println("*************");
    } else {
      println(message);
    }
  }
  
}

String formatMessage(String type, String command) {
   return "kinect:" + type + ":" + command; 
}

// Function to broadcast a PImage over UDP
// Special thanks to: http://ubaa.net/shared/processing/udp/
// (This example doesn't use the library, but you can!)
void broadcast(PImage img) {

  // We need a buffered image to do the JPG encoding
  BufferedImage bimg = new BufferedImage( img.width,img.height, BufferedImage.TYPE_INT_RGB );

  // Transfer pixels from localFrame to the BufferedImage
  img.loadPixels();
  bimg.setRGB( 0, 0, img.width, img.height, img.pixels, 0, img.width);

  // Need these output streams to get image as bytes for UDP communication
  ByteArrayOutputStream baStream  = new ByteArrayOutputStream();
  BufferedOutputStream bos    = new BufferedOutputStream(baStream);

  // Turn the BufferedImage into a JPG and put it in the BufferedOutputStream
  // Requires try/catch
  try {
    ImageIO.write(bimg, "jpg", bos);
  } 
  catch (IOException e) {
    e.printStackTrace();
  }

  // Get the byte array, which we will send out via UDP!
  byte[] packet = baStream.toByteArray();

  // Send JPEG data as a datagram
  println("Sending datagram with " + packet.length + " bytes");
  try {
    println("Trying to send...");
    //ds.send(new DatagramPacket(packet,packet.length, InetAddress.getByName("localhost"),clientPort));
    ds.send(new DatagramPacket(packet,packet.length, InetAddress.getByName("131.252.240.131"),clientPort));
  } 
  catch (Exception e) {
    e.printStackTrace();
  }
}
