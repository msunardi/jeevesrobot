#!/usr/bin/env python
""" Unit tests for the base_control_node module.
"""
import logging
import unittest

from geometry_msgs.msg import Twist

import base_control_node as bc

class BaseControllerTransformHandlerTest(unittest.TestCase):
    def test_twist_to_wheel_velocities(self):
        pass
    
    def test_wheel_velocities_to_twist(self):
        #expected result
        should_be = Twist()
        should_be.linear.x = 1.0
        should_be.linear.y = 0.0
        should_be.angular.z = 0.0

        th = bc.BaseTransformHandler(1.0, 0.5, 0.5)
        w = (1.0, 1.0, 1.0, 1.0)        
        twist = th.wheel_velocities_to_twist(w)
        self.assertEqual(twist, should_be)
    
        w_out = th.twist_to_wheel_velocities(twist)
        self.assertEqual(w_out, w)
        
        
def main():
    """Run all tests."""
    logging.basicConfig(level=logging.INFO)
    unittest.main()


if __name__ == "__main__":
    main()