import rospy
from jeeves_2d_nav.srv import *

class ROSClients:

    def get_qrcode(self):
         
        rospy.wait_for_service('qrcode_pos_srv')
         
        qrcode_pos_h = rospy.ServiceProxy('qrcode_pos_srv', qrcode_pos_service)                       
         
        return qrcode_pos_h().json_resp
     
    def get_battery_status(self):
        status = rospy.wait_for_message('/battery_status', BatteryStatus, 20)
        return status

