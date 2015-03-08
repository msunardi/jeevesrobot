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

FILTER_LEN = 50
class BatteryMonitor(threading.Thread):
    def __init__(self):
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        self.publisher = rospy.Publisher("/battery_status", BatteryStatus, queue_size=5)
        self.A_buffer = deque()
        self.V_buffer = deque()
        threading.Thread.__init__(self)

    def __del__(self):
        self.serial_port.close()
        
    def run(self):
        while not rospy.is_shutdown():
            # a line looks like this:
            # "A0:0.6000, A1:4.6000\n"
            line = self.serial_port.readline()
            tokens = line.split(',')
            if len(tokens) < 2:
                continue
            A0 = tokens[0]
            A1 = tokens[1]
            # parse
            try:
                A0 = float(A0[A0.find(':') + 1 :])
                A1 = float(A1[A1.find(':') + 1 :])
            except:
                rospy.logwarn("Warning: exception while parsing string from current sensor.")        
                continue
            
            # convert from sensor units to voltage: the AndyMark am2709 current probe, connected
            # to A0, outputs 40mV/A, and has a quiescent output of 0.6V
            amps = A0
            amps -= 0.59
            amps /= 0.040
            volts = A1
            self.A_buffer.append(amps)
            self.V_buffer.append(volts)
            if len(self.A_buffer) < FILTER_LEN:
                continue
            self.A_buffer.popleft()
            self.V_buffer.popleft()
            
            # publish
            battery_status = BatteryStatus()
            battery_status.amps = np.mean(self.A_buffer)
            battery_status.volts = np.mean(self.V_buffer)
            self.publisher.publish(battery_status)
            
def main(args):
    rospy.init_node('battery_monitor_node', anonymous=True, log_level=rospy.INFO)
    monitor = BatteryMonitor()
    monitor.start()
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
