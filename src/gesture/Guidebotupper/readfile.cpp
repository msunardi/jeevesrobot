
/*
Cody Hanks
cody@byterule.com for support
Readfile.cpp
uses Readfile.h
functions for opening and parsing a scene command file
*/

#include "readfile.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;
//Default constructor accepts non null filename
// opens and reads file into array of strings and the lentgh to private length
Readfile::Readfile(string filename)
{
    //cout<<filename<<endl;
    char *buffer;
    ifstream reader(filename.c_str());
    if(reader.is_open())
    {
         // get length of file:
          reader.seekg (0, ios::end);
          int length = reader.tellg();
          reader.seekg (0, ios::beg);

          buffer = new char[length];

          // read data as a block:
          reader.read (buffer,length);
          reader.close();
          m_text= string(buffer);
          //cout<<Splitstrings()<<endl;
          Splitstrings();
    }
    else
        cout<<"Error opening file";

}
//default destructor
Readfile::~Readfile()
{
    //dtor
}

/*
Process the file (Gestures pointer to Gestures objexct
                  Speech pointer to speech object)
    this methood to parse the list of strings previously read into movements and speech and submit the
    actions to each seperate object.
*/
void Readfile::process(Gestures *objGestures)
{
    //for loop, on length of read items pulls all items and cycles through the parsed strings
    for(int a =0; a<m_arraylength;a++)
    {
        //check for frist char as char or text
        if(!isdigit((m_splittext[a].c_str())[0]))
        {
            //send text items to the speak command
            //if(!(m_splittext[a]=="\n") and m_splittext[a].length()>2)
                //objSpeech->speak(m_splittext[a]);
            //cout<<"text"<<m_splittext[a]<<endl;
        }
        else
        {
            //send commands to the Gestures in Threaded process
            //cout<<"Command"<<m_splittext[a]<<endl;
            string servo = m_splittext[a].substr(0,m_splittext[a].find("_",0));
                string temp = m_splittext[a].substr(m_splittext[a].find("_",0)+1,m_splittext[a].length());
            string value = temp.substr(0,temp.find("_",0));
            string delay = temp.substr(temp.find("_",0)+1, temp.length());

            objGestures->tSetServo(servo::servo_name(objGestures->servolist[atoi(servo.c_str())].Getservoid()),atoi(value.c_str())*4,atoi(delay.c_str()));
        }
    }
}

/*
    Thead pull the split strings out of the text file
*/
int Readfile::Splitstrings()
{
    //create a temp array
    string *temparray;
    //use temp array to fill from string splits.
    temparray= new string[count(m_text.begin(), m_text.end(), '%')+2];
    //trouble shooting
    //cout<<count(m_text.begin(), m_text.end(), '%')+2<<endl;
    temparray = recursivefindsplit(0,m_text,temparray);
    //gedt count for array items
    int a=0;
    while (!temparray[a].empty())
        a++;
    //create class level array
    m_splittext = new string[a+1];
    m_arraylength = a+1;
    //copy items to new array
    for (int index =0 ; index <a+1;index++)
    {
        m_splittext[index] = temparray[index];
    }
    return 0;
}

///re-curseive call to read all elemnts to their own box
string* Readfile::recursivefindsplit(int count, string text, string *array)
{
    //trouble shooting
    //cout<<text<<endl;
    //cout<<count<<endl;
    // check for empty string
    if(text.empty())
    {
        return array;
    }
    // check that length of text is >1
    if(text.length()>1)
    {
        //find % sign in the string
        if(text.find("%",0)>1)
        {
            //split to sub strings and send remainder back to recursion
            string substring = text.substr(0,text.find("%",0));
            array[count]=substring;
            if(substring.length()==text.length())
            {
                return array;
            }
            string remainingtext = text.substr(text.find("%",0),text.length());
            // cout<<count<<" "<<array[count]<<endl;
            count++;
            return recursivefindsplit(count,remainingtext,array);
        }
        else
        {
            //the % is first remove and send to recursion
            string substring = text.substr(1,text.length());
            return recursivefindsplit(count,substring,array);


        }
    }
    //theres no % symbol or text return
    else
        return array;
}
