import threading
import time

import rospy
import smach
import smach_ros
from greeter.srv import *

import tour_guide_states as tgs


def init_state_machine():
    # create the top-level state machine
    sm = smach.StateMachine(outcomes=['idle'])
    with sm:
        smach.StateMachine.add(
                'GREETING', GreetingState(),
                transitions={'tour_requested': 'GIVING_TOUR'})

        # GIVING_TOUR container
        sm_giving_tour = smach.StateMachine(outcomes=['AT_HOME'])
        # add sub-states
        with sm_giving_tour:
            smach.StateMachine.add(
                'IN_TRANSIT', tgs.InTransitState(),
                transitions={'arrived_at_poi': 'GIVING_SPIEL',
                             'arrived_at_home': 'AT_HOME'})
            smach.StateMachine.add(
                'GIVING_SPIEL', tgs.GivingSpielState(),
                transitions={'spiel_complete': 'IN_TRANSIT'})

        smach.StateMachine.add(
            'GIVING_TOUR', sm_giving_tour,
            transitions={'AT_HOME': 'GREETING'})
    return sm


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
    def __init__(self, sm):
        self.sleeper = rospy.Rate(1)
        self.sm_top = sm
        rospy.Service('/greeter/begin_tour',
                      BeginTour,
                      self.handle_begin_tour)
        threading.Thread.__init__(self)

    def handle_begin_tour(self, req):
        return 0

    def run(self):
        while not rospy.is_shutdown():
            self.sleeper.sleep()


if __name__ == '__main__':
    rospy.init_node('greeter_executive_node')

    # create the state machine
    sm_top = init_state_machine()

    # kick off the Greeter, which monitors the state machine
    greeter = Greeter(sm_top)
    greeter.start()

    # Start an IntrospectionServer so we can examine
    # the running state machine
    sis = smach_ros.IntrospectionServer('sm_viewer_server', sm_top, '/SM_ROOT')
    sis.start()

    # start the top-level state machine
    sm_top.execute()
    rospy.spin()
