#!/usr/bin/env python
import rospy
import threading
import time
from std_msgs.msg import String as FaceCmd

# this is a node that is used to publish messages to the
# face_expression topic

FACE_COMMAND_SEND_RATE_HZ = 6 # 10th of a second period
class DemoFacePublisher(threading.Thread):
    def __init__(self):
        self.sleeper = rospy.Rate(FACE_COMMAND_SEND_RATE_HZ)
        self.command = rospy.Publisher('/face_expression', FaceCmd, queue_size = 5)
        threading.Thread.__init__(self)

    def run(self):
        #publish some commands with some time constraints
        self.sleeper.sleep()

        # grrrrr
        time.sleep(3) #sleep for a bit between expresion commands
        self.command.publish('topdown')
        time.sleep(3)
        self.command.publish('squint')
        time.sleep(3)
        self.command.publish('botup')
        time.sleep(3)
        self.command.publish('angler')
        time.sleep(3)
        self.command.publish('curve')
        time.sleep(3)

        rospy.loginfo('DemoFacePublisher.run(): is exiting.')

def main(args):
    rospy.init_node('expression_demo_pub_node', anonymous = True, loginfo =rospy.INFO)
    # create an instance of the class, start it up, and spin
    demo = DemoFacePublisher()
    demo.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
