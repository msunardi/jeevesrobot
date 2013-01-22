# Maisee Brown 
# 11/27/2012
# SendAngles.py
# This script reads the animation angles while an animation is running and sends it serially to the Arduino Uno on serial port 3. The Arduino then controls the servos with this information. This script operates all 6 motors needed for the 6 degrees of freedom in my robotic arm.

# This assumes pySerial is installed
# Also, this script is highly inefficient. The serial port is 
# opened and closed EVERY time it is updated...but it works. :)

from armInclude import *
from math import degrees
import site 
import serial

# Create a handler that is called when the animation window changes frames
def frameChangeHandler(scene):
    
    # declare variables to access the x, y, and z angles of the euler rotations
    x = 0
    y = 1
    z = 2
    
    # Set up and open the serial port we will use to communicate to the Arduino
    ser = serial.Serial(port="COM3",baudrate=115200)
    
    # get the frame number we are currently on
    i = bpy.context.scene.frame_current
    
    # only sending every other frame so mod I to check if this is the frame to send or ignore
    if(i%2 == 1):
        
        # verify that we are in pose mode so that we get the correct angles
        bpy.ops.object.mode_set(mode='POSE')
        
        #debug info
        print(i)
    
        # Read the euler rotations of the angles on this frame
        WristAngles = PoseHand.rotation_euler
        ElbowAngles = PoseLowerArm.rotation_euler
        ShoulderAngles = PoseUpperArm.rotation_euler
        UpperMostAngle = PoseShoulderRotation.rotation_euler
        
        WristLift = WristAngles[x]
        ElbowTwist = ElbowAngles[y]
        ElbowLift = ElbowAngles[x]
        ShoulderTwist = ShoulderAngles[y]
        ShoulderLift = ShoulderAngles[z]
        ShoulderSwing = UpperMostAngle[y]

        # Convert the radians given above into degrees
        WristLiftDeg = math.degrees(WristLift)
        ElbowTwistDeg = math.degrees(ElbowTwist)
        ElbowLiftDeg = math.degrees(ElbowLift)
        ShoulderTwistDeg = math.degrees(ShoulderTwist)
        ShoulderLiftDeg = math.degrees(ShoulderLift)
        ShoulderSwingDeg = math.degrees(ShoulderSwing)
        
        print(ShoulderTwistDeg)
        
        # Apply necessary offsets for each of the degrees of freedom
        # The servo motors only understand angles between 0 and 180.
        # These offsets will put the middle of the angles range at the 
        # 90 degree so that the servo can trotate to the center of it's 
        # range of freedom. These can be adjusted if design needs to be changed.
        WristLiftDeg += 90
        ElbowTwistDeg += 90
        ElbowLiftDeg += 120
        ShoulderTwistDeg += 90
        ShoulderLiftDeg += 170
        ShoulderSwingDeg += 180
        ShoulderSwingDeg = 180 - ShoulderSwingDeg +10
        ElbowLiftDeg = 120- ElbowLiftDeg
        WristLiftDeg = 180 - WristLiftDeg
        
        
        # Convert the floating point angles above into ints
        WristLiftInt = int(WristLiftDeg)
        ElbowTwistInt = int(ElbowTwistDeg)
        ElbowLiftInt= int(ElbowLiftDeg)
        ShoulderTwistInt = int(ShoulderTwistDeg)
        ShoulderLiftInt = int(ShoulderLiftDeg)
        ShoulderSwingInt = int(ShoulderSwingDeg)
        
        # Convert the ints into strings, andd due to the algorithm used on
        # the Arduino, add a newline character at the end of the string so that the
        # encode function below will not ignore it.
        WristLiftStr = "\n"+ str(WristLiftInt)+"\n"
        ElbowTwistStr = str(ElbowTwistInt)+"\n"
        ElbowLiftStr = str(ElbowLiftInt)+"\n"
        ShoulderTwistStr = str(ShoulderTwistInt)+"\n"
        ShoulderLiftStr = str(ShoulderLiftInt)+"\n"
        ShoulderSwingStr = str(ShoulderSwingInt)+"\n"
        
        # Build the serial string with all of the angles above order
        # to send serially over the usb port to the Arduino. 
        serialString = WristLiftStr + ElbowTwistStr + ElbowLiftStr +  ShoulderTwistStr + ShoulderLiftStr + ShoulderSwingStr
        
        # This is kinda obvious
        ser.write(serialString.encode('ascii'))
        
        # debug print statement
        
        print(ShoulderTwistDeg)
        print(serialString.encode('ascii'))
    

#bpy.app.handlers.frame_change_pre.
bpy.app.handlers.frame_change_pre.append(frameChangeHandler)
print("Set Handler")