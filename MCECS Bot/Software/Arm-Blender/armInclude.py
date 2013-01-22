#Maisee Brown
#10/29/2012
#Arm movements collection include file

import bpy, math, mathutils

# define MACROS for the Armature and its edit bones and pose bones so that 
# we can shorten the names used
Armature = bpy.data.objects["Armature"]
Arm = Armature.data
Shoulder = Arm.bones["Shoulder"]
UpperArm = Arm.bones["UpperArm"]
LowerArm = Arm.bones["LowerArm"]
Hand = Arm.bones["Hand"]
PoseHand = Armature.pose.bones["Hand"]
PoseLowerArm = Armature.pose.bones["LowerArm"]
PoseUpperArm = Armature.pose.bones["UpperArm"] 
PoseShoulderRotation = Armature.pose.bones["Shoulder"]
Head = bpy.data.objects["Sphere"]
Body = bpy.data.objects["Cube"]

# number of frames to advance for each key_frame insert 
counterIncrement = 10

# min and max rotation angles for each degree of freedom of the arm
ShoulderRotationMin = -180
ShoulderRotationMax = 0
ShoulderRotationStep = 60
ShoulderLiftMin = -160
ShoulderLiftMax = 0
ShoulderLiftStep = 80
ShoulderTwistMin = -40
ShoulderTwistMax = 40
ShoulderTwistStep = 40
ElbowLiftMax = 0
ElbowLiftMin = -120
ElbowLiftStep = 30
ElbowTwistMax = 90
ElbowTwistMin = -90
ElbowTwistStep = 90
WristLiftMax = 30
WristLiftMin = -90
WristLiftStep = 30

# empty global arrays for filling with angles desired for the full motion
FullSwingShoulder = []
FullTwistShoulder = []
FullLiftShoulder = []
FullLiftElbow = []
FullLiftWrist = []
FullTwistElbow = []

# Characteristics of the robot that are used in the 
# collision detection function checkCoordinates()
WidthOfBody = 2.0
HeigthOfHeadFromShoulder = 3.0
DistanceBetweenBodyAndArm = 1.0
LengthOfArm = 5.7

def setRangesOfMotion():
    global FullSwingShoulder 
    global FullTwistShoulder
    global FullLiftShoulder
    global FullLiftElbow 
    global FullLiftWrist 
    global FullTwistElbow
    
    # Clear existing ranges
    FullSwingShoulder=[]
    FullTwistShoulder=[]
    FullLiftShoulder=[]
    FullLiftElbow=[]
    FullTwistElbow=[]
    FullLiftWrist=[] 
    
    # Set range for smooth motion for full motion
    for i in range(0, ShoulderRotationMax,ShoulderRotationStep):
        FullSwingShoulder.append(i)
    for i in range(ShoulderRotationMax, ShoulderRotationMin, -ShoulderRotationStep):
        FullSwingShoulder.append(i)
    for i in range(ShoulderRotationMin, 1, ShoulderRotationStep):
        FullSwingShoulder.append(i)
 #   for i in range(0, ShoulderLiftMax,ShoulderLiftStep):
#        FullLiftShoulder.append(i)
    for i in range(ShoulderLiftMax, ShoulderLiftMin, -ShoulderLiftStep):
        FullLiftShoulder.append(i)
    for i in range(ShoulderLiftMin, 1, ShoulderLiftStep):
        FullLiftShoulder.append(i)
    for i in range(0, ShoulderTwistMax,ShoulderTwistStep):
        FullTwistShoulder.append(i)
    for i in range(ShoulderTwistMax, ShoulderTwistMin, -ShoulderTwistStep):
        FullTwistShoulder.append(i)
    for i in range(ShoulderTwistMin, 1, ShoulderTwistStep):
        FullTwistShoulder.append(i)
    for i in range(ElbowLiftMax, ElbowLiftMin, -ElbowLiftStep):
        FullLiftElbow.append(i)
    for i in range(ElbowLiftMin, 1, ElbowLiftStep):
        FullLiftElbow.append(i)
    for i in range(0, ElbowTwistMax,ElbowTwistStep):
        FullTwistElbow.append(i)
    for i in range(ElbowTwistMax, ElbowTwistMin, -ElbowTwistStep):
        FullTwistElbow.append(i)
    for i in range(ElbowTwistMin, 1, ElbowTwistStep):
        FullTwistElbow.append(i)
    for i in range(0, WristLiftMax,WristLiftStep):
        FullLiftWrist.append(i)
    for i in range(WristLiftMax, WristLiftMin, -WristLiftStep):
        FullLiftWrist.append(i)
    for i in range(WristLiftMin, 1, WristLiftStep):
        FullLiftWrist.append(i)

def checkCoordinates(Xpos,Ypos,Zpos):
    print("Called checkCoordinates")
    if ((Xpos >= (-0.125)) and (Zpos <= (HeigthOfHeadFromShoulder))and (Ypos < 1) and (Ypos > -1)): 
        #"""and (Ypos < ((WidthOfBody/2))) and (Ypos > -(WidthOfBody/2)) and ((Zpos > -LengthOfArm) )):"""
        #print("Arm is inside the box")
        #print(Xpos)
        #print("Y")
        #print(Ypos)
        #print("Z")
        #print(Zpos) 
        return False 
    else:
        #print("Arm is outside the box")
        #print(Xpos)
        #print("Y")
        #print(Ypos)
        #print("Z")
        #print(Zpos)        
        return True
    
def setRestPosition():
    PoseUpperArm.rotation_euler = (math.radians(0), math.radians(0), math.radians(0))
    PoseShoulderRotation.rotation_euler = (math.radians(0), math.radians(0), math.radians(0))
    PoseLowerArm.rotation_euler = (math.radians(0), math.radians(0), math.radians(0))
    PoseHand.rotation_euler = (math.radians(0), math.radians(0), math.radians(0))
    bpy.ops.anim.change_frame(frame = 0)
    PoseUpperArm.keyframe_insert(data_path="rotation_euler")
    PoseShoulderRotation.keyframe_insert(data_path="rotation_euler")
    PoseLowerArm.keyframe_insert(data_path="rotation_euler")
    PoseHand.keyframe_insert(data_path="rotation_euler") 