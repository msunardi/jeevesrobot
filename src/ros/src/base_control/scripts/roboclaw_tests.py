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

class RoboClawCommTests(unittest.TestCase):
    def setUp(self):
        self.robo_rear = rc.RoboClaw('/dev/ttyACM0', 2400, 250, 3336)
        
    def tearDown(self):
        del self.robo_rear
        
    def test_read_speeds_real_fast(self):
        # lazy pace
        POLL_RATE_HZ = 5
        TEST_LEN_S = 10 # ten-second test
        ITERATIONS = TEST_LEN_S * POLL_RATE_HZ
        
        logging.info("Calling readM1speed() " + str(ITERATIONS) + " times at " + str(POLL_RATE_HZ) + " Hz...")
        speeds = []
        for i in range(ITERATIONS):
            (v, status) = self.robo_rear.readM1speed()
            speeds.append(v)
            time.sleep(1.0 / POLL_RATE_HZ)
        logging.info("Received: " + str(len(speeds)))            
        logging.info("Mean speed: " + str(sum(speeds) / float(len(speeds))))
        time.sleep(1.0)
        
        # way faster
        POLL_RATE_HZ = 30
        TEST_LEN_S = 10 # ten-second test
        ITERATIONS = TEST_LEN_S * POLL_RATE_HZ
        
        logging.info("Calling readM1speed() " + str(ITERATIONS) + " times at " + str(POLL_RATE_HZ) + " Hz...")
        speeds = []
        for i in range(ITERATIONS):
            (v, status) = self.robo_rear.readM1speed()
            speeds.append(v)
            time.sleep(1.0 / POLL_RATE_HZ)
        logging.info("Received: " + str(len(speeds)))            
        logging.info("Mean speed: " + str(sum(speeds) / float(len(speeds))))
        time.sleep(1.0)

        # now do it while interspersing move commands
        POLL_RATE_HZ = 30
        CMD_DIVIDER = 10
        TEST_LEN_S = 10 # ten-second test
        ITERATIONS = TEST_LEN_S * POLL_RATE_HZ
        logging.info("Calling readM1speed() " + str(ITERATIONS) + " times at " +
                     str(POLL_RATE_HZ) + " Hz while sending move commands " + 
                     "every " + str(CMD_DIVIDER) + " reads...")
        speeds = []
        for i in range(ITERATIONS):
            if 0 == (i % CMD_DIVIDER):
                self.robo_rear.SetM1SpeedAccel(1000, 500)
            else:
                (v, status) = self.robo_rear.readM1speed()
            speeds.append(v)
            time.sleep(1.0 / POLL_RATE_HZ)
        self.robo_rear.SetM1SpeedAccel(1000, 0)
        logging.info("Received: " + str(len(speeds)))            
        logging.info("Mean speed: " + str(sum(speeds) / float(len(speeds))))

        POLL_RATE_HZ = 20
        CMD_DIVIDER = 4
        TEST_LEN_S = 60 # ten-second test
        ITERATIONS = TEST_LEN_S * POLL_RATE_HZ
        logging.info("Calling readM1speed() " + str(ITERATIONS) + " times at " +
                     str(POLL_RATE_HZ) + " Hz while sending move commands " + 
                     "every " + str(CMD_DIVIDER) + " reads...")
        speeds = []
        for i in range(ITERATIONS):
            if 0 == (i % CMD_DIVIDER):
                self.robo_rear.SetM1SpeedAccel(1000, 500)
            else:
                (v, status) = self.robo_rear.readM1speed()
            speeds.append(v)
            time.sleep(1.0 / POLL_RATE_HZ)
        self.robo_rear.SetM1SpeedAccel(1000, 0)
        logging.info("Received: " + str(len(speeds)))            
        logging.info("Mean speed: " + str(sum(speeds) / float(len(speeds))))
        
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
        v = self.robo_front.readM1speed()
        self.assertEqual(0, v[0])
        v = self.robo_front.readM2speed()
        self.assertEqual(0, v[0])

        # set it in 'motion' and read the speed
        self.robo_front.SetMixedSpeedAccel(250, 500, 500)
        v = self.robo_front.readM1speed()
        self.assertEqual(500, v[0])
        v = self.robo_front.readM2speed()
        self.assertEqual(500, v[0])

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
        poll_rate_hz = 10
        self.cmd_queue = deque()
        self.output_queue = deque()
        self.mgr = rc.RoboClawManager(ports, baudrate, accel,
                                max_ticks_per_second,
                                rc.TICKS_PER_REV,
                                poll_rate_hz,
                                self.cmd_queue,
                                self.output_queue,
                                True)
        logging.info("Starting RoboClawManager.")
        self.mgr.start()

    def tearDown(self):
        logging.info("RoboClawManagerTests.tearDown():"
                     " waiting for RoboClawManager to stop...")
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
        self.assertAlmostEqual(w1[0], 1.0, delta=0.1)
        self.assertAlmostEqual(w1[1], 1.0, delta=0.1)
        self.assertAlmostEqual(w1[2], 1.0, delta=0.1)
        self.assertAlmostEqual(w1[3], 1.0, delta=0.1)

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