#!/usr/bin/env python
import rospy
import string
import threading
from std_msgs.msg import String
import redis

class FingerRedis(threading.Thread):

    def __init__(self):
        self.pub = rospy.Publisher("/finger_solved", String, queue_size=10)

        r = redis.StrictRedis(host='localhost', port=6379, db=0)
        self.redis_sub = r.pubsub()
	self.old_message = ""
        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(30)

    def redis_handler(self, message):
        out = "Finger_Redis: GOT MESSAGE: %s" % message['data']
        rospy.loginfo(out)
        print(out)

        # if repeated message, ignore
        if message['data'] == self.old_message:
            return
        self.old_message = message['data']
        answer = message['data'].split('is ')[-1]
        try:
            ans = int(answer)
            self.pub.publish(answer)
        except ValueError as e:
            rospy.loginfo("Not a number")
            print("Not a number")


    def run(self):
        self.redis_sub.subscribe(**{'response': self.redis_handler})
        while not rospy.is_shutdown():
            self.redis_sub.get_message()
            self.sleeper.sleep()

def main():
    rospy.init_node('finger_redis', anonymous=True)
    finger_redis = FingerRedis()
    finger_redis.start()
    rospy.spin()

if __name__ == "__main__":
    main()

