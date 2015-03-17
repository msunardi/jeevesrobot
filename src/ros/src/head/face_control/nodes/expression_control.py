#!/usr/bin/env python

import serial
import threading
import rospy
# this is the text command that this node will issue
# it is understood by the firmare running in the face arduino
from std_msgs.msg import String as PrimitiveCommand
CMD_RATE_Hz = 1 # speed at which we will send commands to face controller


class FaceController(threading.Thread):
    def __init__(self):
        self.sleeper = rospy.Rate(CMD_RATE_Hz)
        self.primitive_command_queue = PrimitiveCommand()
        self.expression_sub = rospy.Subscriber("face_expression", PrimitiveCommand, self.send_primitive_cb)  # These are some topics to which I might subscribe and how I would handle them (callback)
        self.FaceCtl = serial.Serial("/dev/FaceDuino", 9600)                                                  # Initialize the face controller's USB port
        threading.Thread.__init__(self)

    def run(self):
        command = PrimitiveCommand()

        while not rospy.is_shutdown():
            self.sleeper.sleep()                                                                              # sleep for a bit to maintain proper face command frequency

            # atomically pop a command off the queue
            with self.lock:
                command = self.primitive_command_queue

            # now send that message to the face firmware
            FaceCtl.write(command)
            # PWL you need to check if this string gets the quotes around it, if that makes sense


        rospy.loginfo("FaceController.run() is exiting; thank you and have a nice day.")

    # this is the callback function that gets run when a
    # topic to which we subscribe has a message posted to it
    def send_primitive_cb(self, PrimitiveCommand):
        # atomically push a command on the queue
        with self.lock:
            self.primitive_command_queue = PrimitiveCommand


def main(args):
    rospy.init_node('face_expression_commands_node', anonymous=True, log_level=rospy.INFO)
    controller = FaceController()
    controller.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

