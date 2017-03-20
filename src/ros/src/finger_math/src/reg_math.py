#!/usr/bin/env python
import rospy
import string
import re
from std_msgs.msg import String

re1='(\\d+)'	# Integer Number 1
re2='.*?'	# Non-greedy match on filler
re3='((?:[a-z][a-z]+))'	# Word 1
re4='.*?'	# Non-greedy match on filler
re5='(\\d+)'	# Integer Number 2

rg = re.compile(re1+re2+re3+re4+re5,re.IGNORECASE|re.DOTALL)
    



def callback(data):
    a = 0
    b = 0
    c1 = ''
    c  = 0
    txt = string.lower(data.data)
    m = rg.search(txt)
    if m:
        a = int(m.group(1))
        c1 = m.group(2)
        b = int(m.group(3))

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if (c1 == 'plus'):
        c = a + b
    elif (c1=='minus'): 
        c = a - b
    
                     
            
    
    pub = rospy.Publisher('finger_solved', String, queue_size = 1)      #, callback_two)
    r   = rospy.Rate(10)  #10hz
    msg = String()
    msg.data = str(c)
        
    pub.publish(msg)
    rospy.loginfo(msg)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("finger_math", anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

