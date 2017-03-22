#!/usr/bin/env python
import sys
import serial
import threading
import random as r
import rospy
from time import sleep
# this is the text command that this node will issue
# it is understood by the firmare running in the face arduino
from std_msgs.msg import String

CMD_RATE_Hz = 1 # speed at which we will send commands to face controller


class FaceController(threading.Thread):
    def __init__(self):
        self.sleeper = rospy.Rate(CMD_RATE_Hz)
        self.expression_sub = rospy.Subscriber("face_command", String, self.face_callback)  # These are some topics to which I might subscribe and how I would handle them (callback)
        self.expression_sub = rospy.Subscriber("lip_sync", String, self.lipsync_callback)  # These are some topics to which I might subscribe and how I would handle them (callback)
	self.pub = rospy.Publisher('/face', String, queue_size=10)
        self.expression = {'neutral': ['g', 'i'],
                           'smile': ['q', 'j'],
                           'happy': ['q', 'j'],
                           'shy': ['w', '1:5', 'o', 'a', '1:5', '6:6'],
                           'sad': ['f', 'w', '3:6', 'i']}
        self.mouth = {'m': 'i', 'o': 'l', 'u': 'k', 'a': 'j', 'e': 'p', 'neutral': ['i', 'o', 'p','j']}
        self.eyes = ['q','w','3:3','2:2','2:5','2:3','3:6','6:4','g']
        threading.Thread.__init__(self)

    def run(self):
        #command = PrimitiveCommand()
        facial = String()
        facial.data = 'i'
        self.pub.publish(facial)

        while not rospy.is_shutdown():
            if r.random() < 0.15:
                facial.data = 'e'
            elif r.random() > 0.5:
                facial.data = r.choice(self.eyes)
            self.pub.publish(facial)

            self.sleeper.sleep()                                                                              # sleep for a bit to maintain proper face command frequency

        rospy.loginfo("FaceController.run() is exiting; thank you and have a nice day.")

    # this is the callback function that gets run when a
    # topic to which we subscribe has a message posted to it
    def face_callback(self, command):
        face_cmd = command.data

        # atomically push a command on the queue
        #with self.lock:
        #    self.primitive_command_queue = PrimitiveCommand
        facial = String()

        self.publish_face(self.expression[face_cmd])
        #for p in self.expression[face_cmd]:
        #    facial.data = p
        #    self.pub.publish(facial)
        #    sleep(0.5)

        sleep(3)
        facial.data = 'g'
        self.pub.publish(facial)
        sleep(0.5)
        facial.data = r.choice(self.mouth['neutral'])
        self.pub.publish(facial)

    def lipsync_callback(self, speech):
        facial = String()
        for s in speech.data:
            seq = self.get_mouth_shape(s)
            self.publish_face(seq)
            sleep(0.1)
        sleep(3)
        facial.data = 'g'
        self.pub.publish(facial)
        sleep(0.5)
        facial.data = r.choice(self.mouth['neutral'])
        self.pub.publish(facial)

    def publish_face(self, seq):
        facial = String()
        for s in seq:
            facial.data = s
            self.pub.publish(facial)
            sleep(0.5)

    def get_mouth_shape(self, s):
        m_data = ['g']
        if s == 'm':
            m_data = [self.mouth[s]]
        elif s in ['p', 'b']:
            m_data = [self.mouth['m'], self.mouth['e']]
        elif s in ['c', 'd', 'e', 'f', 'g', 'h', 'j', 'k', 'l', 'n', 'q', 's', 't', 'x', 'z']:
            m_data = [self.mouth['a']]
        elif s in ['a', 'i']:
            m_data = [self.mouth['e']]
        elif s in ['o']:
            m_data = [self.mouth['o']]
        elif s in ['r', 'u', 'y', 'v', 'w']:
            m_data = [self.mouth['u']]

        return m_data


def main(args):
    rospy.init_node('face_expression_commands_node', anonymous=True, log_level=rospy.INFO)
    controller = FaceController()
    controller.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

