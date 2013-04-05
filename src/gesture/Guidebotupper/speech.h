/*
Cody Hanks
cody@byterule.com for support
Speech.cpp

using festival to generate speech
NOTE: Festival cannot be multithreaded multiple times
*/
#ifndef SPEECH_H
#define SPEECH_H
#include <string>
#include <festival/festival.h>
#include <pthread.h>

class speech
{
    public:
        /** Default constructor */
        speech();

        void init();
        /** Default destructor */
        ~speech();
        /** Access m_Heap
         * \return The current value of m_Heap
         */
        void speak(string text);
        void tspeak(string *text);
        //void *threadcall(void *text);

        int GetHeap() { return m_Heap; }
        /** Set m_Heap
         * \param val New value to set
         */
        void SetHeap(int val) { m_Heap = val; }
        /** Access m_load_initfiles
         * \return The current value of m_load_initfiles
         */
        int Getload_initfiles() { return m_load_initfiles; }
        /** Set m_load_initfiles
         * \param val New value to set
         */
        void Setload_initfiles(int val) { m_load_initfiles = val; }
    protected:
    private:
        int m_Heap; //!< Member variable "m_Heap"
        int m_file;
        int m_load_initfiles; //!< Member variable "m_load_initfiles"
};

#endif // SPEECH_H
