#!/usr/bin/env python
import sys, threading
import random as r
import xml.etree.ElementTree as etree
import decimal

from scipy.interpolate import interp1d, lagrange
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rospkg
import json
from math import *
from std_msgs.msg import Float32, Float64, String
from collections import deque

from poses import Poses

class PagelistReader(threading.Thread):
    roll = 0.0
    pan = 0.0
    then = 0

    def __init__(self):
        # self.yaw_sub = rospy.Subscriber('/head/cmd_pose_yaw', Float32, self.yaw_callback)
        # self.pitch_sub = rospy.Subscriber('/head/cmd_pose_pitch', Float32, self.pitch_callback)
        self.speech_cmd = rospy.Subscriber('/jeeves_speech/speech_command', String, self.speech_cmd_callback)

        # List of recognized joints (note, left_sho_pitch servo doesn't work)
        self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=10)        
        self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=10)
        self.right_sho_pitch = rospy.Publisher('/right_sho_pitch/command', Float64, queue_size=10)
        self.right_sho_roll = rospy.Publisher('/right_sho_roll/command', Float64, queue_size=10)
        self.right_elbow = rospy.Publisher('/right_elbow/command', Float64, queue_size=10)
        self.left_sho_pitch = rospy.Publisher('/left_sho_pitch/command', Float64, queue_size=10)
        self.left_sho_roll = rospy.Publisher('/left_sho_roll/command', Float64, queue_size=10)
        self.left_elbow = rospy.Publisher('/left_elbow/command', Float64, queue_size=10)

        self.joints = {'R_ELBOW': self.right_elbow,
            'R_SHO_PITCH': self.right_sho_pitch,
            'R_SHO_ROLL': self.right_sho_roll,
            'L_ELBOW': self.left_elbow,
            'L_SHO_PITCH': self.left_sho_pitch,
            'L_SHO_ROLL': self.left_sho_roll,
            'HEAD_PAN': self.head_pan_pub,
            'HEAD_TILT': self.head_tilt_pub}

        # Load from .pagelist file
        rospack = rospkg.RosPack()
        tree = etree.parse('%s/src/Chair-Poses.pagelist' % rospack.get_path('arm_gesture'))
        # tree = etree.parse('/home/mathias/Projects/jimmy_ros/src/arbotix_controller/src/PositionSequence.pagelist')
        self.pages = tree.findall('.//PageClass')
        self.page_length = len(self.pages)

        self.action = []    

        # print "Pose classes: %s" % pose_classes
        # R_SHO_PITCH = [[int(pcx.find('R_SHO_PITCH').text) for pcx in pc] for pc in pose_classes]

        # self.poses = Poses(self.pages[0])    # Only pick the first 'page'
        # rospy.loginfo(self.poses.getPoses())

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(50)

    def speech_cmd_callback(self, data):
        jdata = json.loads(data.data)
        cmd = jdata['command']

        if cmd != 'greeting':
            return

        n = r.randint(0, 3)  # Pick random motion

        rospy.loginfo("Got greeting message from speech!")
        xposes = Poses(self.pages[n]) 
        poses = xposes.getPoses()
        timing = xposes.getTiming()

        new_poses = {}
        posex_length = 0

        for joint, pub in self.joints.iteritems():
            posex = poses[joint]
            # rospy.loginfo("Posex[%s]: %s" % (joint, posex))
            # fposex = self.interpolate(posex, timing)
            fposex = self.interpolate2(posex, timing)
            rospy.loginfo("lagrange fPosex[%s]: %s" % (joint, fposex))
            new_poses[joint] = fposex
            posex_length = len(fposex)
                # for p in fposex:
                #     pub.publish(Float64(p))

                #     #  rospy.loginfo("----------------------%s" % rospy.get_time())
                #     while rospy.get_time() - self.then < 0.03:                                         
                #         pass
                #     self.then = rospy.get_time()

        p = len(timing['PauseTime'])
        pause = [t * 0.01 for t in timing['PauseTime']]
        rospy.loginfo("p/pause: %s/%s" % (p, pause))

        for i in range(posex_length):
            for joint, pub in self.joints.iteritems():
                pos = new_poses[joint][i]
                pub.publish(Float64(pos))
            d = 0.02
            mx = posex_length/p + 1
            if (i % mx == 0):
                ind = i/mx
                d += pause[ind] 
            rospy.loginfo("Wait for: %s" % d)
            while rospy.get_time() - self.then < d:                                         
                pass
            self.then = rospy.get_time()
      
    def run(self):
        rospy.loginfo("Setting up JimmyController ...")

        # joints = {'R_ELBOW': self.right_elbow,
        #     'R_SHO_PITCH': self.right_sho_pitch,
        #     'R_SHO_ROLL': self.right_sho_roll,
        #     'L_ELBOW': self.left_elbow,
        #     'L_SHO_PITCH': self.left_sho_pitch,
        #     'L_SHO_ROLL': self.left_sho_roll,
        #     'HEAD_PAN': self.head_pan_pub,
        #     'HEAD_TILT': self.head_tilt_pub}

        # self.poses = Poses(pages[0])    # Only pick the first 'page'
        # rospy.loginfo(self.poses.getPoses())

        l = len(self.pages)

        flag = False
        n = 0
        #xposes = Poses(self.pages[4]) 
        # rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
        # rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())         
        #poses = xposes.getPoses()
        #timing = xposes.getTiming()

        #rospy.loginfo("GPoses: %s" % poses)
        #rospy.loginfo("Timing: %s" % timing)

        # poses_ = self.interpolate(poses, timing)

        # rospy.loginfo("Poses_: %s" % poses_)
        

        while not rospy.is_shutdown():

            # if not flag:
            #     self.action = iter(self.getAction())
            #     flag = True
            # else:
            #     try:
            #         pose = self.action.next()
            #         for pub, pos in pose:
            #             pub.publish(pos)
            #     except StopIteration:
            #         flag = False
            #         self.action = []

            if l > 1:
                n = r.randint(0, l-1)
            else:
                n = 0
            # xposes = Poses(self.pages[n]) 
            # rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
            # rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())         
            # poses = xposes.getPoses()
            # timing = xposes.getTiming()
            # for i in range(len(poses['R_ELBOW'])):
            #     for joint, pub in joints.iteritems():
            #         foo = self.mapp(poses[joint][i])
            #         pub.publish(Float64(foo))

            #     # rospy.loginfo("----------------------%s" % rospy.get_time())
            #     while rospy.get_time() - self.then < 0.3:                                         
            #         pass
            #     self.then = rospy.get_time()

            # n = n % l
            #xposes = Poses(self.pages[n]) 
            # rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
            # rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())         
            #poses = xposes.getPoses()
            #timing = xposes.getTiming()

            # poses_ = self.interpolate(poses, timing)

            # for i in range(len(poses_['R_ELBOW'])):
            #new_poses = {}
            #posex_length = 0

            #for joint, pub in self.joints.iteritems():
            #    posex = poses[joint]
            #    # rospy.loginfo("Posex[%s]: %s" % (joint, posex))
            #    # fposex = self.interpolate(posex, timing)
            #    fposex = self.interpolate2(posex, timing)
            #    rospy.loginfo("lagrange fPosex[%s]: %s" % (joint, fposex))
            #    new_poses[joint] = fposex
            #    posex_length = len(fposex)
                # for p in fposex:
                #     pub.publish(Float64(p))

                #     #  rospy.loginfo("----------------------%s" % rospy.get_time())
                #     while rospy.get_time() - self.then < 0.03:                                         
                #         pass
                #     self.then = rospy.get_time()

            #p = len(timing['PauseTime'])
            #pause = [t * 0.01 for t in timing['PauseTime']]
            #rospy.loginfo("p/pause: %s/%s" % (p, pause))

            #for i in range(posex_length):
            #    for joint, pub in self.joints.iteritems():
            #        pos = new_poses[joint][i]
            #        pub.publish(Float64(pos))

            #    d = 0.02
            #    mx = posex_length/p + 1
            #    if (i % mx == 0):
            #        ind = i/mx
            #        d += pause[ind] 
            #    rospy.loginfo("Wait for: %s" % d)
            #    while rospy.get_time() - self.then < d:                                         
            #        pass
            #    self.then = rospy.get_time()

                # mx = posex_length/p + 1
                # if (i % mx == 0):
                #     ind = i/mx
                #     while rospy.get_time() - self.then < pause[ind] * 0.01:
                #         pass
                #     self.then = rospy.get_time()

                # if (i != 0) and (i % p == 0):                    
                #     ind = int(i/p)
                #     rospy.loginfo("Pause time: %s" % pause[ind])
                #     while rospy.get_time() - self.then < pause[ind]/1000.0:
                #         pass
                #     self.then = rospy.get_time()
            # # n += 1

            self.sleeper.sleep() 

    def getAction(self):
        chosen_action = []
        fpose = []
        if self.page_length > 1:
            n = r.randint(0, self.page_length-1)
        else:
            n = 0
        xposes = Poses(self.pages[n]) 
        rospy.loginfo("**-------\nPlaying page: %s" % xposes.getTitle()) # print title
        rospy.loginfo("**-------\nMotion: %s" % xposes.getPoses())         
        poses = xposes.getPoses()
        timing = xposes.getTiming()

        poses_ = self.interpolate(poses, timing)

        for i in range(len(poses_['R_ELBOW'])):
            for joint, pub in self.joints.iteritems():
                foo = self.mapp(poses_[joint][i])
                fpose.append((pub, Float64(foo)))
            chosen_action.append(fpose)
        # rospy.loginfo(chosen_action)

        return chosen_action

    # def interpolate(self, action, timing):
    #     for t in timing['Time']:    # t * 8 milliseconds, rate: 20Hz (50 ms)
    #         tx = int(t) * 8
    #         steps = tx/50 + 1  # tx milliseconds/50 ms
    #         # rospy.loginfo("Timing: %s (%s ms), Steps: %s" % (t, tx, steps))

    #         if steps < 1.0:
    #             continue

    #         # rospy.loginfo("Action: %s" % action)
    #         for k, poses in action.iteritems():
    #             l = len(poses) - 1
    #             rospy.loginfo("XPoses: [%s] %s" % (k, poses))
    #             rposes = []
    #             for i in range(l):                    
    #                 iposes = []
    #                 p1 = int(poses[i])
    #                 p2 = int(poses[i+1])
    #                 rospy.loginfo("P1: %s, P2: %s" % (p1, p2))
    #                 if p1 == p2:
    #                     for j in range(steps):
    #                         iposes += [p1]
    #                 else:
    #                     rospy.loginfo("FOOOOBAAARRRR")
    #                     dp = p2 - p1
    #                     dpdt = float(dp)/float(steps)
    #                     for j in range(steps):
    #                         iposes.append(int(p1 + j*dpdt))
    #             rospy.loginfo("Iposes[%s]: %s" % (k, iposes))
    #             rposes = iposes
    #             # rospy.loginfo("rposes: %s" % rposes)
    #             action[k] = rposes
    #             rposes = None
    #             # rospy.loginfo("%s: %s" % (k, action[k]))
    #     return action

    def interpolate(self, fubar, timing):
        fb = []
        for i in range(len(fubar)-1):
            f1 = fubar[i]
            f2 = fubar[i+1]
            diff = f2 - f1
            steps = int(timing['Time'][i+1] * 8/50.0)
            rospy.loginfo("Duration/steps: %s/%s" % (timing['Time'][i], steps))
            increment = float(diff)/float(steps)
            print increment
            fx = []
            for j in range(steps+1):
                fx += [self.mapp(f1 + j*increment)]

            print fx
            fb += fx
        print "%s (%s)" % (fb, len(fb))
        return fb

    def interpolate2(self, fubar, timing):
        l = len(fubar)
        x = np.linspace(0, l-1, num=l, endpoint=True)
        y = fubar
        # f = interp1d(x,y)
        f2 = interp1d(x,y, kind='cubic')
        #f2 = lagrange(x, y)
        steps = 0
        for t in timing['Time']:
            steps += int(t * 8/50.0)

        xnew = np.linspace(0, l-1, num=steps, endpoint=True)

        fx = [self.mapp(f) for f in f2(xnew)]

        # rospy.loginfo("Cubic interpolated: %s" % fx)
        return fx
        
        # axes = plt.gca()
        # axes.set_ylim([1200, 3000])
        # # plt.plot(x,y,'o', xnew, f(xnew), '-', xnew, f2(xnew), '--', xnew, f3(xnew), 'x')
        # # plt.plot(x,y,'o', xnew, f(xnew), '-', xnew, f2(xnew), '--', xnew, f3(xnew), '+', xnew, f4(xnew), 's')
        # data, = plt.plot(x,y,'o', label='data')
        # linear, = plt.plot(xnew, f(xnew), '-', label='linear')
        # cubic, = plt.plot(xnew, f2(xnew), '--', label='cubic')
        # lagrang, = plt.plot(xnew, f3(xnew), 'g^', label='lagrange')
        # plt.legend([data, linear, cubic, lagrang])
        # plt.show()


    def mapp(self, x):
        decimal.getcontext().prec=7
        return (x - 512.0)/512.0 * 2.0       

def main(args):
    rospy.init_node('pagelist_reader_node', anonymous=True)
    pr = PagelistReader()
    pr.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)


