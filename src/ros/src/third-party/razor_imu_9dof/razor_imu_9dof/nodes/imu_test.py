#!/usr/bin/env python
import serial
from time import sleep

p = serial.Serial('/dev/ttyUSB1', baudrate=57600, timeout=1.0)

def extract_triplet(s):
     vals = (s[s.find('=') + 1 : s.find('\r')]).split(',')
     return [float(x) for x in vals]
    
def main():
    p = serial.Serial(port='/dev/imu',baudrate=57600, timeout=1.0)
    
    sleep(5) # Sleep for 8 seconds to wait for the board to boot then only write command.
    p.write('#osct' + chr(13)) # 'Calibrated sensor readings'
    p.readline()
    p.readline()
    
    while(True):
        try:            
            s = p.readline()
            print s
            if(s.find('#G') == 0):
                omegas = extract_triplet(s)
                print omegas #x, y, z
            
            if(s.find('#A') == 0):
                accels = extract_triplet(s)
                print accels #x, y, z
                
            if(s.find('#Y') == 0):
                ypr = extract_triplet(s)
                print ypr # yaw, pitch, roll
        except ValueError:
            print "caught ValueError. Moving on."

if __name__ == '__main__':
    main()