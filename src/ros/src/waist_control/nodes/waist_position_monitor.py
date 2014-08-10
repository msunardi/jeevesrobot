#!/usr/bin/env python
import sys
import threading
import serial
import rospy
from waist_control.msg import FeedbackPosition

class WaistPositionPublisher(threading.Thread):
    def __init__(self):
        self.arduino = serial.Serial("/dev/waist_feedback", 9600) #path to Arduino
	self.actuator_1_pos_publisher = rospy.Publisher("actuator_1/position", FeedbackPosition)
	self.actuator_2_pos_publisher = rospy.Publisher("actuator_2/position", FeedbackPosition)
	self.actuator_3_pos_publisher = rospy.Publisher("actuator_3/position", FeedbackPosition)
	self.actuator_4_pos_publisher = rospy.Publisher("actuator_4/position", FeedbackPosition)
	threading.Thread.__init__(self)

    def run(self):
        loop_count = 0
	while not rospy.is_shutdown():
            positions = "empty"
	    while positions.count("_") != 3:     
	        positions = self.arduino.readline()
	    positions_list = positions.split("_")
	    string_posM1 = positions_list[0]	#0 is the first position of the list
            string_posM2 = positions_list[1]
	    string_posM3 = positions_list[2]
	    string_posM4 = positions_list[3]
	    self.posM1 = int (string_posM1)
	    self.posM2 = int (string_posM2)
	    self.posM3 = int (string_posM3)
	    self.posM4 = int (string_posM4)
	    self.actuator_1_pos_publisher.publish(FeedbackPosition(self.posM1))
	    self.actuator_2_pos_publisher.publish(FeedbackPosition(self.posM2))
	    self.actuator_3_pos_publisher.publish(FeedbackPosition(self.posM3))
	    self.actuator_4_pos_publisher.publish(FeedbackPosition(self.posM4))
                                    
def main(args):
    rospy.init_node('waist_position_monitor_node', anonymous=True, log_level=rospy.INFO)
    pub = WaistPositionPublisher()
    pub.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
