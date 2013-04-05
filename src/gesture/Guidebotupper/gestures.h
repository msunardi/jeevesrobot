/*
cody hanks
cody@byterule.com for support
*/



#ifndef GESTURES_H
#define GESTURES_H
#include <string>
#include <pthread.h>
#include "servo.h"
using namespace std;

class Gestures
{
    public:

        const string devicelocation; //!< Member variable "devicelocation"

        /** Default constructor */
        Gestures();

        /** Set servo Method given a servo name and new value **/
        void SetServo(servo::servo_name id,unsigned short value);
        /**  threaded servo set with delay   **/
        void tSetServo(servo::servo_name id,unsigned short value,int delay);

        /** Default destructor */
        ~Gestures();

        /* connect to the device set m_file*/
        int Connect();
        /* Disconnect m_file*/
        void Disconnect();
        /* Default values and Max/Min values */
        void ServoSetup();

        /**
            Threaded servo set command  pointer to pass all related items to the servo set thread

        */
        struct ServoCommand
        {
            int CommandType;
          servo::servo_name id;
          unsigned short value;
          int delay;
          int fd;
          pthread_mutex_t File_Mutex;
          char* Message;

        };

        void tSetServo(Gestures::ServoCommand *thiscommand);

        /** Access m_file
         * \return The current value of m_file
         */
        int Getfile() { return m_file; }
        /** Set m_file
         * \param val New value to set
         */
        void Setfile(int val) { m_file = val; }
        //Servo id list from Servo types
        servo *servolist;
        //multithread file access mutex
        pthread_mutex_t count_mutex;
        //thread call for servo move
    protected:
    private:
        //static const string devicelocation = "/dev/ttyACM0";
        int m_file; //!< Member variable "m_file"

};

#endif // GESTURES_H
