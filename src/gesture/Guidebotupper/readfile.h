
/*
Cody Hanks
cody@byterule.com for support
Readfile.cpp
uses Readfile.h
functions for opening and parsing a scene command file
*/

#ifndef READFILE_H
#define READFILE_H
#include <string>
#include <stdlib.h>
#include <iostream>
#include "gestures.h"
#include "servo.h"
//#include "speech.h"

using namespace std;

class Readfile
{
    public:
        /** Default constructor */
        Readfile(string filename);
        /** Default destructor */
        virtual ~Readfile();
        /** Access m_text
         * return text from read file
         */
        string Getfiletext() { return m_text; }
        /** Access m_split text
         * return current value of m_splittext
         */
        string* GetStringsArray(){return m_splittext;}
        /** Access
         * return current value of m_arraylength
         */
        int GetStringsArrayLength(){return m_arraylength;}
        //process the file data to text and speech
        void process(Gestures *objGestures);
    protected:
    private:
        int Splitstrings();
        string* recursivefindsplit(int, string, string*);
        string m_text; //!< Member variable "m_filename"
        string *m_splittext;
        int m_arraylength;
};

#endif // READFILE_H
