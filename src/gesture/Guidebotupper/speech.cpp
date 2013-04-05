/*
Cody Hanks
cody@byterule.com for support
Speech.cpp

using festival to generate speech
NOTE: Festival cannot be multithreaded multiple times
*/
#include "speech.h"

///constructor
speech::speech()
{
    //run festival init commands
    init();
}
///init the festival system
void speech::init()
{
    //set values and activate the festival system
    m_Heap = 210000;
    m_load_initfiles = 1;
    festival_initialize(m_load_initfiles,m_Heap);
}
///Say the provided text
void speech::speak(string text)
{
    //festival_wait_for_spooler();
	festival_say_text(text.c_str());

}
///destructor
speech::~speech()
{
    //dtor
}
