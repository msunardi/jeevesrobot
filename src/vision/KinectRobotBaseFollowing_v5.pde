/* Rami Alshafi Winter 2013
   Intelegent Rootics II
   Portland State University
   Dr.Perkowski
   Version 5*/
import processing.serial.*;
import SimpleOpenNI.*;
SimpleOpenNI kinect;
Serial port;
PVector handVector = new PVector();
PVector mapHandVector = new PVector();
color handPointColor = color(255,0,0);
int draw=1, Rec=1, crea=1, upda=1, Des=1;
boolean DisBF = true;
boolean roto  = false;
int close;
int closeX;
int closeY;
int send = 2;
boolean      handsTrackFlag = false;

void setup()
{
  kinect = new SimpleOpenNI(this); 
  kinect.setMirror(false);
  kinect.enableDepth();
  kinect.enableGesture();
  kinect.enableHands();
  kinect.addGesture("RaiseHand");
  size (kinect.depthWidth()+200, kinect.depthHeight()+100);
    String portName = Serial.list()[6];
    port = new Serial(this, portName, 57600);
}
void draw()
{
  background(0);
  close = 6000;
  draw++;
  kinect.update();
  kinect.convertRealWorldToProjective(handVector, mapHandVector);
  image(kinect.depthImage(), 0, 100); 
  int [] depthValues = kinect.depthMap();
  int handDistance = int(mapHandVector.x) + (int(mapHandVector.y)*640);
  int millimeters = depthValues[handDistance];
  textSize(20);
  fill(255, 255, 0);
  textAlign(CENTER, CENTER);
  text(millimeters+"\n"+"("+int(mapHandVector.x)+" , "+int(mapHandVector.y)+")", mapHandVector.x, mapHandVector.y);
  for(int y = 0; y<480; y++){
    for(int x=0; x<640;x++){
      int i = x + y*640;
      int currentDis = depthValues[i];
      if(currentDis > 0 && currentDis < close){
        close = currentDis;
        closeX = x;
        closeY = y;
      }
    }
  }

 if(close < 600 && close > 0){
   send = 2;
   port.write(send);
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
 }else if(handsTrackFlag == true)
  {
    if(DisBF == false)
  {
    if (millimeters <= 900 && millimeters > 0){
      if (send != 4){
        send = 4;
        port.write(send);
        textSize(20);
        fill(255, 0, 0);
        textAlign(CENTER, CENTER);
        text("Backword", 700, 20);
      }else{
        textSize(20);
        fill(255, 0, 0);
        textAlign(CENTER, CENTER);
        text("Backword", 700, 20);
        text("Distance satisfied? "+DisBF,110,20);
      }
    }else if (millimeters >= 1200 && millimeters > 0){
      if(send != 3){
        send=3;
        port.write(send);
        textSize(20);
        fill(255, 0, 0);
        textAlign(CENTER, CENTER);
        text("Forward",700,20);
      }else{
        textSize(20);
        fill(255, 0, 0);
        textAlign(CENTER, CENTER);
        text("Forward",700,20);
        text("Distance satisfied? "+DisBF,110,20);
      }
    }else if (millimeters < 1200 && millimeters > 900){
        DisBF = true;
        roto = false;
        textSize(20);
        fill(0, 255, 0);
        textAlign(CENTER, CENTER);
        text("Distance satisfied? "+DisBF,110,20);
      if (send != 2){
        send =2;
        port.write(send);
            textSize(20);
            fill(0,255,0);
            textAlign(CENTER, CENTER);
            text("Stay",700,20);
      }else{
            textSize(20);
            fill(0,255,0);
            textAlign(CENTER, CENTER);
            text("Stay",700,20);
      }
    }
 }else if(roto == false){
  if (mapHandVector.x <= 0.25*kinect.depthWidth()){ 
    if (send != 0){
      send = 0;
      port.write(send);
      textSize(20);
      fill(255, 0, 0);
      textAlign(CENTER, CENTER);
      text("Left",700,20);
    }else{
      textSize(20);
      fill(255, 0, 0);
      textAlign(CENTER, CENTER);
      text("Left",700,20);
      text("Rotation satisfied? "+roto,340, 20);
    }
  }else if (mapHandVector.x >= 0.75*kinect.depthWidth()){ 
    if (send != 1){
      send = 1;
      port.write(send);
      textSize(20);
      fill(255, 0, 0);
      textAlign(CENTER, CENTER);
      text("Right",700,20);
  }else{
      textSize(20);
      fill(255, 0, 0);
      textAlign(CENTER, CENTER);
      text("Right",700,20);
      text("Rotation satisfied? "+roto,340, 20);
  }
  }else if (mapHandVector.x <= 0.75*kinect.depthWidth() && mapHandVector.x >= 0.25*kinect.depthWidth()){
            roto = true;
            DisBF = false;
            fill(0,255, 0);
            textAlign(CENTER, CENTER);
            text("Rotation satisfied? "+roto,340, 20);
          if (send != 2){
            send = 2;
            port.write(send);
            textSize(20);
            fill(0,255,0);
            textAlign(CENTER, CENTER);
            text("Stay",700,20);
          }else{
            textSize(20);
            fill(0,255,0);
            textAlign(CENTER, CENTER);
            text("Stay",700,20);
          }

  }
}
  }else{
                
            textSize(20);
            fill(255);
            textAlign(CENTER, CENTER);
            text("Raise\na Hand\nTo start\n Session",720,70);
  }
}
void onRecognizeGesture(String strGesture, PVector idPosition, PVector endPosition)
{
  Rec++;
  kinect.removeGesture(strGesture);
  kinect.startTrackingHands(endPosition);
}
void onCreateHands(int handId, PVector pos, float time)
{
  crea++;
  handsTrackFlag = true;
  handVector = pos;
}
void onUpdateHands(int handId, PVector pos, float time)
{
  upda++;
  handVector = pos;
}
void onDestroyHands(int handId,float time)
{
  Des++;
  handsTrackFlag = false;
  port.write(2);
  kinect.addGesture("RaiseHand");
}
void keyPressed(){
  if (key == 'x') { 
  exit();
  }
}
