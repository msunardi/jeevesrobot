""" Unit tests for the RoboClaw and RoboClawSim classes.
"""
from collections import deque
import copy
import logging
import numpy
import pdb
import time
import unittest

import roboclaw as rc


class RoboClawSimTests(unittest.TestCase):
    def setUp(self):
        self.robo_front = rc.RoboClawSim('/dev/ttyUSB0', 2400, 250, 3336)

    def test_init(self):

        # Sample RoboClaw setup.
        # Roboclaw needs to know what the encoders will read
        # at 100% duty cycle. We generally don't muck with the PID constants.
        # 3336 was empirically determined and is accurate as of 2/2014.
        (p, i, d, q) = self.robo_front.readM1pidq()
        self.robo_front.SetM1pidq(p, i, d, 3336)

    def test_set_and_read_speed(self):

        # should be zero at start
        v = self.robo_front.readM1instspeed()
        self.assertEqual(0.0, v)
        v = self.robo_front.readM2instspeed()
        self.assertEqual(0.0, v)

        # set it in 'motion' and read the speed
        self.robo_front.SetMixedSpeedAccel(250, 500, 500)
        v = self.robo_front.readM1instspeed()
        self.assertEqual(500 / 125.0, v)
        v = self.robo_front.readM2instspeed()
        self.assertEqual(500 / 125.0, v)

    def test_read_version(self):
        ver = "front controller version: " + self.robo_front.readversion()
        logging.info(ver)


class RoboClawManagerTests(unittest.TestCase):
    def setUp(self):
        """start a RoboClawManager"""
        ports = ('/dev/ttyUSB0', '/dev/ttyACM0')
        baudrate = 2400
        accel = 250
        max_ticks_per_second = 3336
        poll_interval_s = 0.1
        self.cmd_queue = deque()
        self.output_queue = deque()
        self.mgr = rc.RoboClawManager(ports, baudrate, accel,
                                max_ticks_per_second,
                                rc.TICKS_PER_REV,
                                poll_interval_s,
                                self.cmd_queue,
                                self.output_queue,
                                True)
        logging.info("Starting RoboClawManager.")
        self.mgr.start()

    def tearDown(self):
        logging.info("Waiting for RoboClawManager to stop...")
        self.mgr.quit = True
        self.mgr.join()

    def test_velocity_output(self):
        # let some zero entries get into the output
        time.sleep(0.5)

        # vector of wheel angular velocities, in rad/s, in order of
        # lf, rf, rr, lr
        w = (1.0, 1.0, 1.0, 1.0)
        self.cmd_queue.append(w)
        time.sleep(0.5)

        # at this point, we should have a bunch of zeros, then some ones
        w = copy.copy(self.output_queue)
        w0 = w.popleft()
        w1 = w.pop()
        self.assertEqual(w0, (0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(w1[0], 1.0)
        self.assertAlmostEqual(w1[1], 1.0)
        self.assertAlmostEqual(w1[2], 1.0)
        self.assertAlmostEqual(w1[3], 1.0)

        w = (0.0, 0.0, 0.0, 0.0)
        self.cmd_queue.append(w)
        time.sleep(0.5)
        w = copy.copy(self.output_queue)
        w2 = w.popleft()
        w3 = w.pop()
        self.assertEqual(w2, (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(w3, (0.0, 0.0, 0.0, 0.0))

    def test_ticks_to_radians(self):
        theta = self.mgr.ticks_to_radians(rc.TICKS_PER_REV)
        self.assertAlmostEqual(theta, 2.0 * numpy.pi)

    def test_radians_to_ticks(self):
       n = self.mgr.radians_to_ticks(2.0 * numpy.pi)
       self.assertEqual(n, rc.TICKS_PER_REV)

def main():
    """Run all tests."""
    logging.basicConfig(level=logging.INFO)
    unittest.main()


if __name__ == "__main__":
    main()