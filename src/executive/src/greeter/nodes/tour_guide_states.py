#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time


# define state Foo
class InTransitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_at_poi', 'arrived_at_home'])
        self.counter = 0

    def execute(self, userdata):
        self.counter += 1
        time.sleep(5)
        if self.counter <= 3:
            return 'arrived_at_poi'
        else:
            rospy.loginfo('arrived_at_home')
            self.counter = 0
            return 'arrived_at_home'


# define state Bar
class GivingSpielState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['spiel_complete'])

    def execute(self, userdata):
        time.sleep(5)
        return 'spiel_complete'

