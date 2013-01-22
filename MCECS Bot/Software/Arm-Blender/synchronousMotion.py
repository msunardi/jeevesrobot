# Maisee Brown
# 10/26/2012
# Full Motion Simulation of Arm with 6 degrees of freedom
# This python script was written to display the full motion of an armature in blender.
# This armature is representation of an arm that will be mechanically built for a robot. This script is intented to show the range of motion that the arm with 6 degrees of freedom will have. This script intends to move all motors simulttaneously in the range of the full motion while checking for collision with the body

import bpy, math
from armInclude import *
    
def simultaneousMotors(counter):
    # Set mode to pose mode
    bpy.ops.object.mode_set(mode='POSE')
    #start frame counter at 0
   # counter = 1
    bpy.ops.anim.change_frame(frame = counter)
    # set up arrays of angles
    SimSwingShoulder = [ 0, -18, -36, -54, -72, -90, -108, -126, -144, -162, -180, -162, -144, -126, -108, -90, -72, -54, -36, -18, 0]
    SimLiftShoulder = [0, -16, -32, -48, -64, -80, -96, -112, -128, -144, -160, -144, -128, -112, -96, -80, -64, -48, -32, -16, 0]
    SimTwistShoulder = [0, -8, -16, -24, -32, -40, -32, -24, -16, -8 ,0, 8, 16, 32, 40, 32, 24, 16, 8, 0, 0]
    SimLiftElbow = [0,-12, -24, -36, -48, -60, -72, -84, -96, -108, -120, -108, -96, -84, -72, -60, -48, -36, -24, -12, 0 ]
    SimTwistElbow = [0, -15, -30, -60, -90, -60, -30, -15,  0, 15, 30, 45, 60, 75, 90, 75, 60, 45, 30, 15,  0]
    SimLiftWrist = [0, -9, -18, -27, -36, -45, -54, -63, -72, -81, -90, -81, -72, -63, -54, -45, -36, -27, -18, -9, 0]
    SimSync = range(0,21)
    for i in SimSync:
        # set new rotation angles
        PoseHand.rotation_euler = (math.radians(SimLiftWrist[i]), math.radians(0), math.radians(0))
        PoseUpperArm.rotation_euler = (math.radians(0), math.radians(SimTwistShoulder[i]), math.radians(SimLiftShoulder[i])) 
        PoseShoulderRotation.rotation_euler = (math.radians(0), math.radians(SimSwingShoulder[i]), math.radians(0))
        PoseLowerArm.rotation_euler = (math.radians(SimLiftElbow[i]), math.radians(SimTwistElbow[i]), math.radians(0))
        # verify that this motion doesn't hit the body or the head
        # if it hits the body continue to next motion without adding a frame
                
        handTailPosition = PoseHand.tail 
        handCenterPosition = PoseHand.center 
        handHeadPosition = PoseHand.head
        lowerHeadPosition = PoseLowerArm.head
        lowerCenterPosition = PoseLowerArm.center
        upperCenterPosition = PoseUpperArm.center
                
        PoseUpperArm.keyframe_insert(data_path="rotation_euler")
        PoseShoulderRotation.keyframe_insert(data_path="rotation_euler")
        PoseLowerArm.keyframe_insert(data_path="rotation_euler")
        PoseHand.keyframe_insert(data_path="rotation_euler")
                            
        print("HandTail=",PoseHand.tail)
        print("HandCenter=",PoseHand.center)
        #print("HandHead=",PoseHand.head)
        #print("LowerHead=",PoseLowerArm.head)
        #print("LowerCenter=",PoseLowerArm.center)
        #print("UpperHead=",PoseUpperArm.head)
        #print("UpperCenter=",PoseUpperArm.center)             
        print (i)
                           
        # check hand first
        if (not (checkCoordinates(handTailPosition[0], handTailPosition[1], handTailPosition[2]))):
            print("HandTail hit body",counter)
            bpy.ops.anim.keyframe_delete()
                    
        elif(not (checkCoordinates(handHeadPosition[0], handHeadPosition[1], handHeadPosition[2]))):
            print( "HandHead hit body", counter)
            bpy.ops.anim.keyframe_delete()
                    
        elif(not (checkCoordinates(handCenterPosition[0], handCenterPosition[1], handCenterPosition[2]))):
            print("HandCenter hit body", counter)
            bpy.ops.anim.keyframe_delete()
                                
        elif(not (checkCoordinates(lowerHeadPosition[0], lowerHeadPosition[1], lowerHeadPosition[2]))):
            print("LowerArmHead hit body", counter)
            bpy.ops.anim.keyframe_delete()
                                
        elif(not (checkCoordinates(lowerCenterPosition[0], lowerCenterPosition[1], lowerCenterPosition[2]))): 
                                
            print("LowerArm hit body", counter)
            bpy.ops.anim.keyframe_delete()

        else:
                            #otherwise add the frame          
                            #insert key frame so we can mark this position in our animation
                        
                           # set frame to new counter value
            bpy.ops.anim.change_frame(frame = counter)                                                    # increment counter so the animation progresses in time
            print(counter)    
            counter = counter + 30
    bpy.context.scene.frame_end = counter
    bpy.ops.anim.change_frame(frame = 0)


def main():
    startcounter = 0
    print("Starting...")
    #bpy.ops.object.mode_set(mode='OBJECT')
    #bpy.ops.object.select_by_type(extend = False, type='ARMATURE')
    bpy.context.scene.frame_start = 0
    setRestPosition()
    setRangesOfMotion()
    #Clear out the actions from any previous run
    bpy.context.object.animation_data.action = None
    bpy.ops.action.new() 
    simultaneousMotors(startcounter)
    # play the animation we just built
    bpy.ops.screen.animation_play()

main()