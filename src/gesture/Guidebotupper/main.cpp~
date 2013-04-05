/*
Cody Hanks
cody@byterule.com for support
ECE 479 Robotics
Guidebot upper body using linux this program. Will read given text file as a scene as for input.
and output as both text and speech.  uses Festival library
Compile with flags "-l Festival -leststring -lestools -lestbase"
include the folders "-I/usr/include/festival -I/usr/lib/speech_tools/include"
Four class files, Readfile, Gestures, Servo and Speech

Servo is used as an object in the Gestures group,  Detailed settings for each servo.
    Servo.h location has the Enum for Servo and ID
Gestures has threaded and non-threaded methoods for setting servos
Device for the servo set "/dev/ttyACM0" --set this in Gestures.cpp


3/29/2013
    Added named pipe and removed all speech.


*/

#include <iostream>
#include <string>
#include <stdlib.h>
#include "servo.h"
#include "gestures.h"
#include "readfile.h"
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#define fifofile "/var/www/servopipe"
using namespace std;
void clear(char* buffer);
int CommandParse(string command, Gestures::ServoCommand * Command, Gestures *objGestures);
int main()
{
    char cbuf[160] ={'\0'};
    FILE * stream;
    string readvalue;
    int chk;
    int rfifo;
    Gestures::ServoCommand * objCommand;
    /*
    Setup class objects for all classes
    three Main objects Speech sets festival values uses current thread "Festival not multithread safe"
    ReadFile "One object per scene file" by creating the object with file location it opens and parses
        the file to seperate strings the format of "Text%servoid_servosetvalue_Delaybeforeaction%Text"
    Gestures provides a list of all servos and thier ids and settings.
    currently program only works from 0-18 servo's on one board.
    */

    //chech that the piped input exists
    if((chk=open(fifofile,O_RDWR))!= -1)
    {
        close(chk);
    }
    else
    {
        //create when the named pipe doesnt exist set the file properties to 7 7 7 basically a free for all
        // this is currently not a real danger but could be later
        if(rfifo =mkfifo(fifofile, 0777)<0)
        {
            cout<<"error createing named pipe"<<endl;
            exit(-1);
        }
    }

    //Gestures object
    Gestures *thisgestures = new Gestures;
        //in gestures object setup servo list
        //this defines values of the max/min and servo ID's from 0 to 18
        thisgestures->ServoSetup();

    thisgestures->SetServo((servo::servo_name)0,(unsigned short)1500*4);
    thisgestures->SetServo((servo::servo_name)0,(unsigned short)992*4);
    thisgestures->tSetServo((servo::servo_name)0,(unsigned short)1500*4,10);
    thisgestures->tSetServo((servo::servo_name)0,(unsigned short)992*4,20);

    //file exists now we open it to read and start to read lines as they are written.
    //we open with r+ so that we can clear the file after reading
    stream = fopen(fifofile, "r");
    //check that stream is not null
    if((stream)!=NULL)
    {
        objCommand  = new Gestures::ServoCommand;
        //while loop for repeated reading from the pipe, close program and stop reading occures when the readValue = close
        while(0!= strcmp(readvalue.c_str(),"close"))
        {
            if(fgets(cbuf,160,stream)!= NULL)
            {
                readvalue = cbuf;
                readvalue.erase(readvalue.find_last_not_of(" \n\r\t\0")+1);
                if(readvalue != "close"){
                    if(CommandParse(readvalue,objCommand,thisgestures)==0)
                    {
                        //cout<<objCommand->CommandType<<":"<<objCommand->id<<":"<<objCommand->value<<":"<<objCommand->delay<<endl;
                        thisgestures->tSetServo(objCommand->id,objCommand->value,objCommand->delay);
                        //send to gestures here
                        //create new pointer
                        //cout<<objCommand->CommandType<<":"<<objCommand->id<<":"<<objCommand->value<<":"<<objCommand->delay<<endl;
                        delete(objCommand);
                        objCommand = new  Gestures::ServoCommand;


                    }
                    else
                    {
                        cout<<"command parse failed please check format"<<endl;
                    }
                }
                //cout<<"cbuf: "<<readvalue<<strlen(readvalue.c_str())<<":"<<endl;
                clear(cbuf);
            }
        }

        fclose(stream);
    }

    exit(0);
}
///Clears buffer after every read so that we dont get multiple extra chars.
void clear(char* buffer)
{
    int a;
    for(a=0;a<160;a++)
    {
        buffer[a] ='\0';
    }
}

int CommandParse(string command, Gestures::ServoCommand * Command, Gestures * objGestures)
{
    try{
    Command->CommandType = atoi(command.substr(0,command.find("_",0)).c_str());
    command = command.substr(command.find("_",0)+1,command.length());
    switch(Command->CommandType)
    {
        //Case 0 Command 0 of type  servoID_Value_Delay
        case 0:
        //get servo id   servo::servo_name(objGestures->servolist[atoi(servo.c_str())].Getservoid())
            Command->id= (servo::servo_name)((unsigned char)atoi(command.substr(0,command.find("_",0)).c_str()));
            //strip off servo ID
            command = command.substr(command.find("_",0)+1,command.length());

        //get value
            Command->value = atoi(command.substr(0,command.find("_",0)).c_str())*4;
            //strip off
            command = command.substr(command.find("_",0)+1,command.length());
        //get delay
            Command->delay = atoi(command.c_str());
        break;
            //Case 1 command 1 type ServoID_value_delay_ServoID_value_delay...repeating up to 160 chars
        case 1:
            while(command.find("_",0)!=string::npos)
            {

                //get servo id   servo::servo_name(objGestures->servolist[atoi(servo.c_str())].Getservoid())
                Command->id= (servo::servo_name)((unsigned char)atoi(command.substr(0,command.find("_",0)).c_str()));
                //strip off servo ID
                command = command.substr(command.find("_",0)+1,command.length());
                //get value
                Command->value = atoi(command.substr(0,command.find("_",0)).c_str())*4;
                //strip off
                command = command.substr(command.find("_",0)+1,command.length());
                //get delay
                Command->delay = atoi(command.c_str());
                //strip delay
                command = command.substr(command.find("_",0)+1,command.length());

                objGestures->tSetServo(Command->id, Command->value,Command->delay);
            }
        break;
    }
        return 0;
    }
    catch(...)
    {
        return -1;
    }
}


