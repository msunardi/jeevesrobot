import time

import rospy
import smach
import smach_ros

import src.executive.src.greeter.modules.tour_guide as tg
from greeter.srv import *


class GreetingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tour_requested'])

        rospy.Service('/greeter/begin_tour',
                      BeginTour,
                      self.begin_tour)

    def begin_tour(self, req):
        pass

    def execute(self, userdata):
        time.sleep(5)
        return 'tour_requested'


def main():
    rospy.init_node('greeter_executive_node')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['idle'])

    # Open the container
    with sm_top:
        smach.StateMachine.add('GREETING', GreetingState(),
                               transitions={'tour_requested': 'GIVING_TOUR'})

        # Create the sub SMACH state machine
        sm_giving_tour = smach.StateMachine(outcomes=['AT_HOME'])

        # Open the container
        with sm_giving_tour:
            # Add states to the container
            smach.StateMachine.add('IN_TRANSIT', tg.InTransitState(),
                                   transitions={'arrived_at_poi': 'GIVING_SPIEL',
                                                'arrived_at_home': 'AT_HOME'})
            smach.StateMachine.add('GIVING_SPIEL', tg.GivingSpielState(),
                                   transitions={'spiel_complete': 'IN_TRANSIT'})

        smach.StateMachine.add('GIVING_TOUR', sm_giving_tour,
                               transitions={'AT_HOME': 'GREETING'})

    # Execute plan
    sis = smach_ros.IntrospectionServer('sm_viewer_server',
                                        sm_top,
                                        '/SM_ROOT')
    sis.start()
    sm_top.execute()
    rospy.spin()

if __name__ == '__main__':
    main()