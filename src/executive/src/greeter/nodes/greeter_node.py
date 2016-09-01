import threading
import time

import rospy
import smach
import smach_ros

import src.executive.src.greeter.modules.tour_guide as tg
from greeter.srv import *


class GreetingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tour_requested'])
        self.tour_requested = False

    def begin_tour(self):
        self.tour_requested = True

    def execute(self, userdata):
        while not self.tour_requested:
            print("Smiling and awaiting a tour request.")
            time.sleep(5)
        self.tour_requested = False  # re-arm
        return 'tour_requested'


class Greeter(threading.Thread):
    def __init__(self):
        self.states = {}
        self.sleeper = rospy.Rate(1)
        rospy.Service('/greeter/begin_tour',
                      BeginTour,
                      self.handle_begin_tour)
        threading.Thread.__init__(self)

        # create the top-level state machine
        self.sm_top = smach.StateMachine(outcomes=['idle'])
        with self.sm_top:
            self.states['greeting_state'] = smach.StateMachine.add(
                'GREETING', GreetingState(),
                transitions={'tour_requested': 'GIVING_TOUR'})

            # GIVING_TOUR container
            sm_giving_tour = smach.StateMachine(outcomes=['AT_HOME'])
            # add sub-states
            with sm_giving_tour:
                smach.StateMachine.add(
                    'IN_TRANSIT', tg.InTransitState(),
                    transitions={'arrived_at_poi': 'GIVING_SPIEL',
                                 'arrived_at_home': 'AT_HOME'})
                smach.StateMachine.add(
                    'GIVING_SPIEL', tg.GivingSpielState(),
                    transitions={'spiel_complete': 'IN_TRANSIT'})

            smach.StateMachine.add(
                'GIVING_TOUR', sm_giving_tour,
                transitions={'AT_HOME': 'GREETING'})

        # Start an IntrospectionServer so we can examine
        # the running state machine
        sis = smach_ros.IntrospectionServer('sm_viewer_server', self.sm_top,
                                            '/SM_ROOT')
        sis.start()

        # start the top-level state machine
        self.sm_top.execute()

    def handle_begin_tour(self, req):
        self.states['greeting_state'].begin_tour()
        return 0

    def run(self):
        self.sleeper.sleep()
        while not rospy.is_shutdown():
            pass


if __name__ == '__main__':
    rospy.init_node('greeter_executive_node')
    greeter = Greeter()
    greeter.start()
    rospy.spin()
