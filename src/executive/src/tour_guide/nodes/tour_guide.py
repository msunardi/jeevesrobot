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
#        rospy.loginfo('Executing state IN_TRANSIT, trip number {0}'
#                      .format(self.counter))
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


class GreetingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tour_requested'])

    def execute(self, userdata):
        time.sleep(5)
        return 'tour_requested'


def main():
    rospy.init_node('tour_guide_executive_node')

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
            smach.StateMachine.add('IN_TRANSIT', InTransitState(),
                                   transitions={'arrived_at_poi': 'GIVING_SPIEL',
                                                'arrived_at_home': 'AT_HOME'})
            smach.StateMachine.add('GIVING_SPIEL', GivingSpielState(),
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
