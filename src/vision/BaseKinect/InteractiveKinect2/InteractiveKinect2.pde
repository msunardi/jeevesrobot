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
//int LEFT = 108, RIGHT = 106, STOP= 107, FORWARD = 105, BACKWARD = 44; // integer varialbles to host commands protocol with Arduino
int LEFT = 97, RIGHT = 100, STOP= 120, FORWARD = 119, BACKWARD = 115, STRAFE_LEFT = 122, STRAFE_RIGHT = 99; // integer varialbles to host commands protocol with Arduino
int BoxX = 740, BoxW = 50,  HlineX1 = 665, HlineX2= 815, VlineX1 = 740, VlineX2 = 740;  // Horizontal variables for robot base animation
int BoxY = 500, BoxH = 50, HlineY1 = 500, HlineY2 = 500, VlineY1 = 425, VlineY2 = 575;  // vertical variables for robot base animation
String song = "Play Music"; // String variable hosting the music button display, set to play music at start
int send = 4; // integer variable to host the outgoing command to serial
boolean handsTrackFlag = false; // Hand tracking flag, set to false at start since we are not tracking the hand at first

int savedTime;
int totalTime = 5000;

//============== Client variables ========
String messageFromServer = "";
String previousType = "";
String previousCmd = "";

//============== setup function =========//
void setup()
{
  String portName = "/dev/ttyACM3";
  port = new Serial(this, portName, 9600); // initialize the serial object, selected port and buad rate
  kinect = new SimpleOpenNI(this); // initialize the kinect object
  kinect.setMirror(true); // Mirror the depth image
  kinect.enableDepth(); // enable the depth camera of the kinect
  kinect.enableGesture(); // enable the Gesture class of the Kinect
  kinect.enableHands(); // enable the hands class of the kienct
  kinect.addGesture("RaiseHand"); // add the RaiseHand Gesture
  minim = new Minim(this); // initialize the Minim object
  size (kinect.depthWidth()+200, kinect.depthHeight()+100); // display the depth image and extra space for User Interface
    //String portName = Serial.list()[6]; // Select the Serial port number CUSTIMIZE!!!!!!!!!!!!! this is hardware specific 
  //String portName = "/dev/ttyACM0";
  //port = new Serial(this, portName, 9600); // initialize the serial object, selected port and buad rate
  player = minim.loadFile("/home/mcecsbot/Music/i wanna love ya.mp3"); // Load the music file, MUST BE IN THE SKETCH FOLDER to be loaded!!
  savedTime = millis();  //start internal timer, counts in milliseconds
  
  client = new Client(this, "127.0.0.1", 8008);
  client.write("iam:kinect");
}
//======================== Main Function=============//
void draw()
{
  int passedTime;
  background(0); // clean the background with black color
  close = 6000; // set the closest point to 6000 mm as a starting point 
  kinect.update(); // update the kinect 
  image(kinect.depthImage(), 0, 100); // display the depth image starting from X=0, Y=100
  int [] depthValues = kinect.depthMap();// Get pixels depth values
  
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
   println("STOP, obstcle  "+send); // print the sent value to the console for checking, (unnecessary but useful for debuging)
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
   rect(closeX, closeY, 85, 30);
   textAlign(CENTER);
   textSize(20);
   fill(255, 0, 0);
   textAlign(CENTER, CENTER);
   text("OBSTCLE",closeX, closeY);
   rectMode(CORNER);
   fill(255, 255, 0);
   rect(4, 40, 600, 40);
   textAlign(CENTER, CENTER);
   fill(255, 0, 0);
   text("obstcle on the way, CANNOT follow you anymore!!!", 300, 50);
   rectMode(CENTER);
   fill(255, 0, 0);
   stroke(255,255, 0);
   strokeWeight(10);
   rect(BoxX, BoxY, BoxW, BoxH);
   line(HlineX1, HlineY1, HlineX2, HlineY2);
   line(VlineX1, VlineY1, VlineX2, VlineY2);
   strokeWeight(1);
   //========== end of warning display ====================//
 } else if(handsTrackFlag == true) {  // Check if we are tracking the hand?
   
   if (previousCmd != "trackhand") {
     client.write(formatMessage("status","trackhand"));
     client.write(formatMessage("arm","wave"));
     previousCmd = "trackhand";
   }
   // if so, get its information
   savedTime = millis();                // reset millis time
   kinect.convertRealWorldToProjective(handVector, mapHandVector); // convert hand position coordinates to projective
   ellipse(mapHandVector.x, mapHandVector.y, 50, 50); // draw a circle on the hand position

   //========== get Hand Distance ============//
   int handDistance = int(mapHandVector.x) + (int(mapHandVector.y)*640); // use the hands X-Y coordinates to select a pixel
   int millimeters = depthValues[handDistance]; // read the depth value of that pixel on which is the hand
   //========== end of getting hand distance =========//
   textSize(20);
   fill(255, 255, 0);
   textAlign(CENTER, CENTER);
   // a choice to display the hand position and distance instead of a circle
   //text(millimeters+"\n"+"("+int(mapHandVector.x)+" , "+int(mapHandVector.y)+")", mapHandVector.x, mapHandVector.y);
   // ============= hand has been detected, inform the user (UI) ==========//
            textSize(20);
            fill(255);
            textAlign(CENTER, CENTER);
            text("Session is ON \n The Robot \n should be \n following \n your hand\nNOW", 720, 170);
            rectMode(CENTER);
            fill(255);
            stroke(255);
            rect(BoxX, BoxY, BoxW, BoxH);
            line(HlineX1, HlineY1, HlineX2, HlineY2);
            line(VlineX1, VlineY1, VlineX2, VlineY2);
            rectMode(CENTER);
            rect(300, 145, 110, 32);
            textAlign(CENTER, CENTER);
            fill(255,0,0);
            text(song, 300, 145);
//============== end of display ========================//
            if (mapHandVector.x > 280 && mapHandVector.x < 320 && mapHandVector.y > 129 && mapHandVector.y < 161){
              // check if the hand is on the music button we created in the display?
                if(!player.isPlaying()){// if so, check if the music is not playing
                       player.play();// if so, play music
                       song = "Pause Music";// replace music button text with "Pause" instaed of "Play"

                   }else if (player.isPlaying()){// otherwise, check if the music is playing
                     player.pause();// if so, pause the music
                     song = "Play Music";// replace music button text with "Play" instaed of "pause"

                   }//otherwise, resume execution
            }// otherwise, resume executon
            
    if(roto == false) {// check if the rotation satisfaction is not satisfied
      if (mapHandVector.x < 0.25*kinect.depthWidth()) { // if so, check if the hand is in the left
        if ((send != RIGHT) && (send != STRAFE_RIGHT)) {// if so, check if we have alrwady  sent this command
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
          textSize(20);
          fill(255, 0, 0);
          textAlign(CENTER, CENTER);
          text("Right",700,20);
    
        } else {// otherwise, we do not need to re-send the command, just display to the user to inform 
        //=========== LEFT display======================//
          textSize(20);
          fill(255, 0, 0);
          textAlign(CENTER, CENTER);
          text("Left",700,20);
          text("Rotation satisfied? "+roto,340, 20);
          rectMode(CENTER);
          fill(255, 136, 0);
          stroke(255);
          rect(BoxX, BoxY, BoxW, BoxH);
          line(HlineX1, HlineY1, HlineX2, HlineY2);
          line(VlineX1, VlineY1, VlineX2, VlineY2);
          stroke(255, 0, 0);
          strokeWeight(20);
          line(BoxX, BoxY, HlineX1, HlineY1);
          strokeWeight(1);
          //======= end of LEFT display =========//
        }
      } else if (mapHandVector.x > 0.75*kinect.depthWidth()) { // otherwise, check if hand is inthre right side
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
          textSize(20);
          fill(255, 0, 0);
          textAlign(CENTER, CENTER);
          text("Left",700,20);
        } else {//otherwise, we do not need to re-send the command
        //============ RIGHT display==============//
            textSize(20);
            fill(255, 0, 0);
            textAlign(CENTER, CENTER);
            text("Right",700,20);
            text("Rotation satisfied? "+roto,340, 20);
            rectMode(CENTER);
            fill(255, 136, 0);
            stroke(255);
            rect(BoxX, BoxY, BoxW, BoxH);
            line(HlineX1, HlineY1, HlineX2, HlineY2);
            line(VlineX1, VlineY1, VlineX2, VlineY2);
            stroke(255, 0, 0);
            strokeWeight(20);
            line(BoxX, BoxY, HlineX2, HlineY2);
            strokeWeight(1);
            //======= end of RIGHT display==========//
        }
      } else if (mapHandVector.x <= 0.75*kinect.depthWidth() && mapHandVector.x >= 0.25*kinect.depthWidth()) { 
        // otherwise, check if the hand is in the middle
                roto = true;// if so, set the rotation flag to true, so it is not executed again before executing the Distance satisfation
                DisBF = false;// also, set the distance satisfaction flag to false so it can be executed next
                fill(0,255, 0);
                textAlign(CENTER, CENTER);
                text("Rotation satisfied? "+roto,340, 20);
              if (send != STOP) {// check if we have already sent this command 
                send = STOP;// if not, set the send variables to STOP to be sent
                port.write(send);// send it
                println("stay "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
                client.write(formatMessage("base","stop"));
                //==== display the command====//
                textSize(20);
                fill(0,255,0);
                textAlign(CENTER, CENTER);
                text("Stay",700,20);
              } else {// otherwise, we do not need to re-send the command, just inform the user
              //================= STOP display=================//
                textSize(20);
                fill(0,255,0);
                textAlign(CENTER, CENTER);
                text("Stay",700,20);
                rectMode(CENTER);
                fill(0, 255, 0);
                stroke(255);
                rect(BoxX, BoxY, BoxW, BoxH);
                line(HlineX1, HlineY1, HlineX2, HlineY2);
                line(VlineX1, VlineY1, VlineX2, VlineY2);
              }// end of STOP display
      }// otherwise, this is unharmful error, resume execution
    } else if(DisBF == false) {// otherwise, check if the distance satisfaction flag is false
      
      if (millimeters < 900 && millimeters > 0) {// if so, check if the hand is close. execlude the 0! its NOISE!!
        if (send != BACKWARD){// if so, check if we have already 
          send = BACKWARD;// if not, set the send variable to move BACKWARD to be sent
          port.write(send);// send it
          println("BACKWARD "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base","reverse"));
          //====== display the backaward command============//
          textSize(20);
          fill(255, 0, 0);
          textAlign(CENTER, CENTER);
          text("Backward", 700, 20);
        }else{// otherwise, we do not need to re-send the back ward command again
        //====================== BACKWARD display=====================//
          textSize(20);
          fill(255, 0, 0);
          textAlign(CENTER, CENTER);
          text("Backward", 700, 20);
          text("Distance satisfied? "+DisBF,110,20);
          rectMode(CENTER);
          fill(255, 136, 0);
          stroke(255);
          rect(BoxX, BoxY, BoxW, BoxH);
          line(HlineX1, HlineY1, HlineX2, HlineY2);
          line(VlineX1, VlineY1, VlineX2, VlineY2);
          stroke(255, 0, 0);
          strokeWeight(20);
          line(BoxX, BoxY, VlineX2, VlineY2);
          strokeWeight(1);
          //=============== end of BACKWARD display ====================//
        }
      } else if (millimeters > 1200 && millimeters > 0) {//otherwise, check if the hand is far. execlude the 0! it is NOISE!!!
        if(send != FORWARD){// chekc if we have already sent this command
          send= FORWARD;// if not, set the send variable to move forward to be sent
          port.write(send);// send it
          println("FORWARD "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base","forward"));
          //======= display the FORWARD command============//
          textSize(20);
          fill(255, 0, 0);
          textAlign(CENTER, CENTER);
          text("Forward",700,20);
        }else{//otherwise, we do not need to resend the FORWARD command again
        //============== just display the FORWARD command ======================//
          textSize(20);
          fill(255, 0, 0);
          textAlign(CENTER, CENTER);
          text("Forward",700,20);
          text("Distance satisfied? "+DisBF,110,20);
          rectMode(CENTER);
          fill(255, 136, 0);
          stroke(255);
          rect(BoxX, BoxY, BoxW, BoxH);
          line(HlineX1, HlineY1, HlineX2, HlineY2);
          line(VlineX1, VlineY1, VlineX2, VlineY2);
          stroke(255, 0, 0);
          strokeWeight(20);
          line(BoxX, BoxY, VlineX1, VlineY1);
          strokeWeight(1);
          //=========== end of FORWARD command =====================//
        }
      } else if (millimeters < 1200 && millimeters > 900) {// check if the hand is in the middle
        DisBF = true;// if so, set the distance satisfaction flag to true, so it does not get executed again before the rotation satisfaction
        roto = false;// also, set the rotation satisfaction flag to false so it get executed next
        textSize(20);
        fill(0, 255, 0);
        textAlign(CENTER, CENTER);
        text("Distance satisfied? "+DisBF,110,20);
        if (send != STOP){// check if we have already sent this command 
          send = STOP;// if not, set the send variable to STOP to be sent
          port.write(send);// send it
          println("stay "+send);//print the sent value to the console for checking, (unnecessary but useful for debuging)
          client.write(formatMessage("base","stop"));
              textSize(20);
              fill(0,255,0);
              textAlign(CENTER, CENTER);
              text("Stay",700,20);
        }else{//otherwise, we do not need to re-send the STOP command again
        //============= Display the STOP command ========================//
              textSize(20);
              fill(0,255,0);
              textAlign(CENTER, CENTER);
              text("Stay",700,20);
              rectMode(CENTER);
              fill(0, 255, 0);
              stroke(255);
              rect(BoxX, BoxY, BoxW, BoxH);
              line(HlineX1, HlineY1, HlineX2, HlineY2);
              line(VlineX1, VlineY1, VlineX2, VlineY2);
              //============= end of STOP display =====================//
        }
      }// other wise, this is unharmful error, resume execution
    }// otherwise, distance satisfied, resume execution
  } else { // otherwise, the hand is not being tracked. could be the begining of session, or hand is lost. Display instruction to detect hand  
    textSize(20);
    fill(0,255,0);
    textAlign(CENTER, CENTER);
    text("random walk",600,20);
    if (previousCmd != "idle") {
      client.write(formatMessage("base","idle"));
      previousCmd = "idle";
    }    
    passedTime = millis() - savedTime;            // time passed since the the program was started. 
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
    
    textSize(20);
    fill(255);
    textAlign(CENTER, CENTER);
    text("Raise\na Hand\nTo start\n Session",720,70);
    rectMode(CENTER);
    fill(0);
    stroke(255);
    rect(BoxX, BoxY, BoxW, BoxH);
    line(HlineX1, HlineY1, HlineX2, HlineY2);
    line(VlineX1, VlineY1, VlineX2, VlineY2);
    //========= end if instructions display================//
     if (player.isPlaying()){// check if the music is playing
             player.rewind();// if so, rewind it, nobody is here :/
     }//otherwise, music not playing, just resume execution
  }
}// return to the begining (End of draw loop)

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
            text("Stay",700,20);
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
