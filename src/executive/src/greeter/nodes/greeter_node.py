import threading
import time

import rospy
import smach
import smach_ros
from std_msgs.msg import Empty

from greeter.srv import *

import tour_guide_states as tgs


def greeter_should_terminate_cb(outcome_map):
    if outcome_map['CMD_MONITOR'] == 'invalid':
        return True
    return False


def greeter_terminating_cb(outcome_map):
    # the only outcome that currently exists
    return 'tour_requested'


def cmd_monitor_cb(user_data, msg):
    return False


def init_state_machine():
    greeter = smach.Concurrence(
        outcomes=['tour_requested'],
        default_outcome='tour_requested',
        child_termination_cb=greeter_should_terminate_cb,
        outcome_cb=greeter_terminating_cb)
    with greeter:
        smach.Concurrence.add(
            'GREETING', GreetingState())
        smach.Concurrence.add(
            'CMD_MONITOR', smach_ros.MonitorState('greeter/begin_tour',
                                                  Empty,
                                                  cmd_monitor_cb))

    # the top-level state machine
    sm = smach.StateMachine(outcomes=['idle'])
    with sm:
        # initial state
        smach.StateMachine.add('GREETER', greeter,
                               transitions={'tour_requested': 'TOUR_GUIDE'})

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
            'TOUR_GUIDE', sm_giving_tour,
            transitions={'AT_HOME': 'GREETER'})
    return sm


class GreetingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tour_requested'])
        self.tour_requested = False

    def begin_tour(self):
        self.tour_requested = True

    def execute(self, userdata):
        while not self.tour_requested:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            print("Smiling and awaiting a tour request.")
            time.sleep(5)
        self.tour_requested = False  # re-arm
        return 'tour_requested'


# class Greeter(threading.Thread):
#     def __init__(self, sm):
#         self.sleeper = rospy.Rate(1)
#         self.sm_top = sm
#         rospy.Service('/greeter/begin_tour',
#                       BeginTour,
#                       self.handle_begin_tour)
#         threading.Thread.__init__(self)
#
#     def handle_begin_tour(self, req):
#         return 0
#
#     def run(self):
#         while not rospy.is_shutdown():
#             self.sleeper.sleep()


if __name__ == '__main__':
    rospy.init_node('greeter_executive_node')

    # create the state machine
    sm_top = init_state_machine()

    # Start an IntrospectionServer so we can examine
    # the running state machine
    sis = smach_ros.IntrospectionServer('sm_viewer_server', sm_top, '/SM_ROOT')
    sis.start()

    # start the top-level state machine
    sm_top.execute()
    rospy.spin()
    sis.stop()
