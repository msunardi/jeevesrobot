#!/usr/bin/env python

# Code to move the servo motors located on the head of Mr. Jeeves
# Code phase 1: The code will call functions from the class Arbotix code written by Mathias S.
#

import rospy
import arbotix
import serial
import sys
from std_msgs.msg import String

# Global parameters to set the co-ordinates of the face
g_yaw_crdnt = 0
g_ptch_crdnt = 0

class Prnt:
    def __init__(self, name):
        print ("Hello ", self.name)
        
class HeadCtrl:
    serial = None       # code segment taken from M.S. code
    # this init will move the head to the default position
    def __init__(self):
        # the class will require the parameter of motion to move the neck
        # the movement can be UP, DOWN, LEFT, RIGHT, CLOCK, ANTI-CLOCK
        # initialize the servo ID to 0 # assuming that all the servo ID will not be 0
        self.servo_speed = [80,80,80]       # set the speed of the servo motors at the safe speed 
        self.servo_crdnt = [500, 500, 500]  # array of servo ID for x, y and z axis control # default values for the position on power on
        self.servo_id = [1, 2, 3]           # the servo id are set to the given ID's by default
        self.servo_resp_chkr = 0            # it will range from 1 to 7 in binary form, Indicate that the servo has responded
        self.yaw_crdnt = 0
        self.ptch_crdnt = 0
        #rospy.subscriber("face_detect_node", Crdnt_x, Crdnt_y, callback)  # use the face_detect_node from where the co-ordinates are read
        # instatantiate the serial port 
        self.serial_port = serial.Serial("/dev/ttyUSB0", baudrate=38400, timeout=1.0)
        print("Serial Port Selected")
        # instantiate the  arbotix class
        # give the serial port opened to the class
        # the serial open, close will be handled by the arbotix class
        self.abtx = arbotix.Arbotix(self.serial_port) # the serial port is a parameter required by the Arbotix class
        print("Arbotix class instantiated")

        resp = []
        resp = self.abtx.get_servo_id(1)     # read the servo id response of servo 1
        print "Response: %s " % resp
        resp = self.abtx.get_servo_id(2)     # read the servo id response of servo 
        print "Response of S2: %s " % resp
        self.abtx.get_servo_id(3)     # read the servo id response of servo 
        print "Response of S3: %s " % resp

        print "Read the Current Position of the Servo"
        resp = self.abtx.get_servo_position(1)
        print "Servo 1 position: ", resp
        resp = self.abtx.get_servo_position(2)
        print "Servo 2 position: ", resp
        resp = self.abtx.get_servo_position(3)
        print "Servo 3 position: ", resp

        print "Set the Servo position to default at power on"
        print "Set the Servo speed to 80"
        resp = self.abtx.set_servo_speed(1, 80)
        print "Servo 1 speed set: ", resp
        resp = self.abtx.set_servo_speed(2, 80)
        print "Servo 2 speed set: ", resp
        resp = self.abtx.set_servo_speed(3, 80)
        print "Servo 3 speed set: ", resp
        print "Set the positions"
        resp = self.abtx.set_servo_position(1, 500)
        print "Servo 1 set position: ", resp
        resp = self.abtx.set_servo_position(2, 500)
        print "Servo 2 set position: ", resp
        resp = self.abtx.set_servo_position(3, 500)
        print "Servo 3 set position: ", resp
        print "Set the position of the servo as per the co-ordinates rec"
        # the following code needs to be in main loop
        # trial code removed and placed in callback function 
        #self.set_servo_position(yaw_crdnt, ptch_crdnt)   # send the x and y co-ordintates rec from the camera
    #def __del__(self):
    #   self.serial_port.close()    # close the serial port
    
    def set_servo_position(self, yaw_crdnt_rec, ptch_crdnt_rec):
        self.old_yaw_crdnt_rec = 0
        self.old_ptch_crdnt_rec = 0

        if ((self.old_yaw_crdnt_rec != yaw_crdnt_rec) and (self.old_ptch_crdnt_rec != ptch_crdnt_rec)):
            rsp = []
            self.old_yaw_crdnt_rec = yaw_crdnt_rec
            print "The current co-ordinates of servo"
            ptch_crdnt = self.abtx.get_servo_position(1)
            yaw_crdnt = self.abtx.get_servo_position(3)
            print yaw_crdnt, ptch_crdnt
            
            # set the speed of the servo to avoid fast motion
            self.abtx.set_servo_speed(1, 80)
            self.abtx.set_servo_speed(3, 80)
            
            # check if the same value is received: if yes then ignore
            if ptch_crdnt != ptch_crdnt_rec:
                self.abtx.set_servo_position(3, ptch_crdnt_rec)
            if yaw_crdnt != yaw_crdnt_rec:
                self.abtx.set_servo_position(1, yaw_crdnt_rec)
            resp = GetServoCrdnt()  # read the position of the servo to check if they have reached the position 
            g_yaw_crdnt = resp[0]
            g_prtch_crdnt = resp[1]
        else:
            print"New co-ordinates received are same as old"
        # Needs to return the set co-ordinates
        return yaw_crdnt_rec, ptch_crdnt_rec
                  
    def servo_id_rd(self):
        # read the servo id present in the head of JEEVES
        self.servo_id[1] = self.ScanServoID(self.servo_id[1])   # read the servo id for x-axis 
        print("Servo Read: ", self.servo_id[1])
        
    def servo_crdnt_rd(self, servo_id):
        # read the co-ordinates of the servo
        self.servo_crdnt[1] = self.GetServoCrdnt(self.servo_id[1])  #read the cor=-ordinates of the x-servo
        print("Servo Crdnt of X: ", self.servo_crdnt[1])
    
    def servo_crdnt_set(self, servo_id, servo_crdnt): # set the co-ordinate of the servo 
        # set the co-ordinate
        resp = []
        resp = self.SetServoCrdnt(self.servo_id[1], self.servo_crdnt[1])
        print("Co-ordinate X: ", resp)
        
    def servo_set_speed(self, servo_id, servo_speed):
        # set the speed of the servo motors individually
        resp = []
        resp = self.SetServoSpeed(self.servo_id[1], self.servo_speed)   # funtion will set the speed of the motor
        print("Servo ID: %d, Speed Set: %d" % (self.servo_id, self.servo_speed))
    
    def SetServoSpeed(self, servo_id, servo_speed):
        # The speed of the respective servo will be configured in this loop
        abtx.set_servo_speed(servo_id, servo_speed)  # this will set the speed of all the servo motors to the same value
        print("Servo ID: %d, Servo Speed: %d" % (servo_id, servo_speed))
        
    
    def ScanServoID(self, servo_id):
        #self.servo_ID = 0        # the ID will be store in the variable servo_ID
        temp_servo_id = 0               # temporary storage of servo_ID the correct ID will then be stored in the servo_ID variable
        servo_id_cnt = 0
        
        # check: Which is the ID of the servo connected to the machine
        for i in range(0, 3):   # there are 3 servo connected to the neck;; check if they are responsive
            #code
            temp_servo_ID = abtx.get_servo_id(i)    
            #temp_servo_ID = Arbotix.get_servo_id(i)     # the ID will be sent from 0 - 255 and it will be checked which ID the servo communicates
            # the ID which the servo communicates will be stored in the variable 'servo_ID'
            #if(servo_id_cnt < 3):    # check if all the servo id are acquired
            if(temp_servo_ID & 0x800):    #check if the response received is error response
                # error response is received
                print("Invalid Servo ID: ", i)      # print the ID number to which the communication failed
                print("Check if %d ID works" % (i+1))    # check if the next ID will work
            else:
                # communication is successfull
                self.servo_id[servo_id_cnt] = temp_servo_id       # store the correct ID in the servo_ID variable
                print("Communication success: Servo ID ", self.servo_id[servo_id_cnt])        # print the servo ID to which the servo has responded
                servo_id_cnt += 1
                break   # communication is successfull to one servo
            #else:   # all the 3 servo ID are acquired
            #    break
    # return the working servo ID number
    
    def GetServoCrdnt(self):
        # Read the servo co-ordinate
        # This code function will control the movement of the 3 servo available in the head of JEEVES
        # Get the servo initial position
        resp = []
        self.servo_crdnt[1] = abtx.get_servo_position(self.servo_id[1])  # get the position co-ordinates of the y-axis servo
        self.servo_crdnt[2] = abtx.get_servo_position(self.servo_id[2])  # get the position co-ordinates of the z-axis servo
        self.servo_crdnt[3] = abtx.get_servo_position(self.servo_id[2])  # get the position co-ordinates of the x-axis servo
        print("Servo Co-ordinate of X: %d, Y: %d, Z: %d" % (self.servo_crdnt[3], self.servo_crdnt[1], self.servo_crdnt[2]))
        resp = [self.servo_crdnt[3], self.servo_crdnt[1], self.servo_crdnt[2]]
        return resp
    
    def SetServoCrdnt(self, servo_id, servo_crdnt):
        #
        resp = []
        resp = abtx.set_servo_position(self.servo_id, self.servo_crdnt, 3)  # 3 is write command instruction opcode
        print("Servo cmd response ", resp)  # print the response after executing the command

# end of class

# call back function to check the co-ordinates rec from the node and convert them to the binary number acceptable to the servo's
# This function will move the head as per the values rec from the camera
# The values received from the camera node will be in float form
# This value will be converted from float type to decimal which is acceptable to the servo motor
# As per the right hand rule the left motion of head is positive and the right motion is negative,
# Similarly the up motion is positive and the down motion is negative
def head_servo_ctrl(data): 
    rospy.loginfo(rospy.get_caller_id() + ' Req rec: %s', data.data)    # display the data subscribed from the node
    if((data.data[2] < 0) and (data.data[2]) > (-90)):  # Head move to the right 500 - val*5.55
        ptch = 500 - (data.data[2] * 5.55)
    elif((data.data[2] > 0) and (data.data[2] < 90)):   # Head move to the left 500 + val * 5.55
        ptch = 500 + (data.data[2] * 5.55)
        
    if((data.data[1] < 0) and (data.data[1]) > (-10)):  # Head move to the up 500 - val*5.55
        yaw = 500 - (data.data[1] * 5.55)
    elif((data.data[1] > 0) and (data.data[2] < 10)):   # Head move to the down 500 + val * 5.55
        yaw = 500 + (data.data[1] * 5.55)
    # This function will set the head servo to the specified co-ordinates
    set_servo_position(yaw, ptch)   # send the x and y co-ordintates rec from the camera
    val = [((g_yaw_crdnt/5.55)-90), ((g_ptch_crdnt/5.55)-90)]
    #head_ctrl_pub.publish((str)(val))

# Publish to :
# Subscribe from :
# Initialize the head position to the default location 500,500
# Publish the co-ordinates
# Subscribe the co-ordinates in the form of radians


def main():
    head_ctrl = HeadCtrl()  # move the head to the default position i.e. 500, 500 which is in the center
    rospy.init_node('headctrl_node', anonymous=True)
    #head_ctrl_pub = rospy.Publisher('/head/cur_pose', String, queue_size=10)
    #rate = rospy.Rate(10)
    rospy.Subscriber('/head/cmd_pose_rad', String, head_servo_ctrl) # call the callback function 
    rospy.spin()
    



#def main(yaw=g_yaw_crdnt, ptch=g_ptch_crdnt):
#    rospy.init_node('head_controller_node', anonymous = True, log_level = rospy.INFO)   #label this node as head_control_node for the rest of functions in ROS
#    rospy.Subscribe('c_cmd_pose_radians', pitch, yaw, callback)   # subscribe from this node the co-ordinates
#    head_ctrl = HeadCtrl(yaw, ptch);
    
#    ros.spin()  # This function keeps the python from exiting i.e. continous execution of the program
    
    

if __name__ == '__main__':
    main()  
