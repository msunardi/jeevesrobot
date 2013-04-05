/*
Cody Hanks
cody@byterule.com for support
    Defines the Enum and servo positions
*/

#ifndef SERVO_H
#define SERVO_H


class servo
{
    public:

        enum servo_name {Wrist_right=0,Wrist_turn=1,Elbow_Extension=2,Arm_raise=3,
								Arm_twist=5,Shoulder =6,Eyes=8,Head_tilt=9,Mouth=11,
								Middle_finger=12,Birdy=13,Pinky=14,Left_arm=15,Thumb=16,Index=17
								};
        servo();

        ~servo();
        unsigned char Getservoid() { return m_servoid; }
        void Setservoid(unsigned char val) { m_servoid = val; }
        unsigned int Getmax() { return m_max; }
        void Setmax(unsigned int val) { m_max = val; }
        unsigned int Getmin() { return m_min; }
        void Setmin(unsigned int val) { m_min = val; }
        unsigned int GetCurVal() { return m_curval; }
        void SetCurVal(unsigned int val) { m_curval = val; }
    protected:
    private:
        unsigned char m_servoid;
        unsigned int m_curval;
        unsigned int m_max;
        unsigned int m_min;
};

#endif // SERVO_H
