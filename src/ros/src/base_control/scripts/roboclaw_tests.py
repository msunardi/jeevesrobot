""" Unit tests for the RoboClaw and RoboClawSim classes.
"""
from collections import deque
import logging
import time
import unittest

import roboclaw as rc


class RoboClawSimTests(unittest.TestCase):
    def test_init(self):
        robo_front = rc.RoboClawSim('/dev/ttyUSB0', 2400, 3336)
        robo_rear = rc.RoboClawSim('/dev/ttyACM0', 2400, 3336)

    # def test_set_and_read_speed(self):
    #     robo_front = rc.RoboClawSim('/dev/ttyUSB0', 2400, 3336)
    #     robo_front.start()


class RoboClawManagerTests(unittest.TestCase):
    def test_init(self):
        ports = ('/dev/ttyUSB0', '/dev/ttyACM0')
        baudrate = 2400
        max_ticks_per_second = 3336
        poll_interval_s = 0.1
        cmd_queue = deque()
        output_queue = deque()
        mgr = rc.RoboClawManager(ports, baudrate, max_ticks_per_second,
                                 poll_interval_s, cmd_queue, output_queue)
        mgr.start()
        mgr.quit = True
        logging.info("Waiting for RoboClawManager to shut down...")
        mgr.join()

def main():
    """Run all tests."""
    logging.basicConfig(level=logging.INFO)
    unittest.main()


if __name__ == "__main__":
    main()