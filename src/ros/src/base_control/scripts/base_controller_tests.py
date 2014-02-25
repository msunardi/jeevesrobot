#!/usr/bin/env python
""" Unit tests for the base_control_node module.
"""
import logging
import unittest

from geometry_msgs.msg import Twist

import base_control_node as bc

class BaseControllerTransformHandlerTest(unittest.TestCase):
    def test_wheel_velocities_to_twist(self):
        th = bc.BaseTransformHandler(1.0, 0.5, 0.5)

        # x-axis, positive
        w = (1.0, 1.0, 1.0, 1.0)
        should_be = Twist()
        should_be.linear.x = 1.0
        should_be.linear.y = 0.0
        should_be.angular.z = 0.0
        twist = th.wheel_velocities_to_twist(w)
        self.assertEqual(twist, should_be)

        # y-axis, positive
        w = (-1.0, 1.0, -1.0, 1.0)
        should_be.linear.x = 0.0
        should_be.linear.y = 1.0
        should_be.angular.z = 0.0
        twist = th.wheel_velocities_to_twist(w)
        self.assertEqual(twist, should_be)

        # z-axis, positive
        w = (-1.0, -1.0, 1.0, 1.0)
        should_be.linear.x = 0.0
        should_be.linear.y = 0.0
        should_be.angular.z = 1.0
        twist = th.wheel_velocities_to_twist(w)
        self.assertEqual(twist, should_be)

    def test_twist_to_wheel_velocities(self):
        th = bc.BaseTransformHandler(1.0, 0.5, 0.5)

        # x-axis, positive
        twist = Twist()
        twist.linear.x = 1.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        should_be = (1.0, 1.0, 1.0, 1.0)
        w = th.twist_to_wheel_velocities(twist)
        self.assertEqual(w, should_be)

        # y-axis, positive
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 1.0
        twist.angular.z = 0.0
        should_be = (-1.0, 1.0, -1.0, 1.0)
        w = th.twist_to_wheel_velocities(twist)
        self.assertEqual(w, should_be)

        # z-axis, positive
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 1.0
        should_be = (-1.0, -1.0, 1.0, 1.0)
        w = th.twist_to_wheel_velocities(twist)
        self.assertEqual(w, should_be)
        
def main():
    """Run all tests."""
    logging.basicConfig(level=logging.INFO)
    unittest.main()


if __name__ == "__main__":
    main()