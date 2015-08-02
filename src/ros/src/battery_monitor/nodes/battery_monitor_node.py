#!/usr/bin/env python
"""
.. module:: battery_monitor_node
   :synopsis: Node that continuously reads current consumption and battery 
   voltage on Jeeves from the battery monitor via USB, then publishes a
   BatteryStatus message to the /battery_status topic. The battery monitor
   is an Adafruit Teensy running an Arduino sketch that simply echoes
   readings from an AndyMark am-2709 current sensor and voltages from
   the main battery.
"""
from collections import deque
import serial
import sys
import threading

import numpy as np
import rospy

from battery_monitor.msg import BatteryStatus
from reset_teensy_usb import reset_teensy

UPDATE_INTERVAL_ms = 100  # time between updates from the sensor 
FILTER_LEN = 50

def to_amp_hours(amps):
    """Return instantaneous amps converted to amp-hours.
    Formula:
        (amps * UPDATE_INTERVAL_ms) * (1 s / 1000 ms) * (1 min / 60 s) * (1 hr / 60 min)
    """
    Ah = amps * UPDATE_INTERVAL_ms * (1. / 1000) * (1. / 60) * (1. / 60)
    return Ah


class BatteryMonitor(threading.Thread):
    def __init__(self):
        self.serial_port = serial.Serial('/dev/battery_monitor', baudrate=115200, timeout=1)
        self.publisher = rospy.Publisher("/battery_status", BatteryStatus, queue_size=5)
        self.A_buffer = deque()
        self.amps = 0.0
        self.amp_hours = 0.0
        threading.Thread.__init__(self)

    def __del__(self):
        self.serial_port.close()
        
    def run(self):
        while not rospy.is_shutdown():
            # a line looks like this:
            # "\rAMPS: 4.35 A\n"
            line = self.serial_port.readline()

            # parse
            try:
                amps = float(line.split()[1])
            except:
                rospy.logwarn("Warning: exception while parsing string from current sensor.")        
                continue
            
            # buffer
            self.A_buffer.append(amps)
            if len(self.A_buffer) < FILTER_LEN:
                continue
            self.A_buffer.popleft()
            
            # filter
            self.amps = np.mean(self.A_buffer)
            self.amp_hours += to_amp_hours(self.amps)
            
            # publish
            battery_status = BatteryStatus()
            battery_status.amps = self.amps
            battery_status.amp_hours = self.amp_hours
            self.publisher.publish(battery_status)
            
def main(args):
    reset_teensy()
    rospy.init_node('battery_monitor_node', anonymous=True, log_level=rospy.INFO)
    monitor = BatteryMonitor()
    monitor.start()
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
