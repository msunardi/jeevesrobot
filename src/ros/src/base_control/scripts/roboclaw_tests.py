""" Unit tests for the RoboClaw and RoboClawSim classes.
"""
import unittest
import roboclaw as rc


class RoboClawSimTests(unittest.TestCase):
    def test_init(self):
        robo_front = rc.RoboClawSim('/dev/ttyUSB0', 2400, 3336)
        robo_rear = rc.RoboClawSim('/dev/ttyACM0', 2400, 3336)

        robo_front.ResetEncoderCnts()
        robo_rear.ResetEncoderCnts()

def main():

    """Run all tests."""
    unittest.main()


if __name__ == "__main__":
    main()