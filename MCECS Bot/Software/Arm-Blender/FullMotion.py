# Maisee Brown
# 10/26/2012
# Full Motion Simulation of Arm with 6 degrees of freedom
# This python script was written to display the full motion of an armature in blender.
# This armature is representation of an arm that will be mechanically built for a robot. This script is intented to show the range of motion that the arm with 6 degrees of freedom will have. Eventually this script will also check that motion will not collide with the body of the robot and will dynamically set limits on the angles for a future script that will control the physical arm by copying th motions of this script. 

import bpy, math
from armInclude import *
    
def rotateAllExtents(counter):
    # Set mode to pose mode
    bpy.ops.object.mode_set(mode='POSE')
    #start frame counter at 0
   # counter = 1
    bpy.ops.anim.change_frame(frame = counter)
    # rotate shoulder from front to back
    #for i in ShoulderRotateAngles:
    for i in FullSwingShoulder:
        # Lift arm at each intermittent Shoulder rotation angle
        for j in FullLiftShoulder:
            # Twist arm at each intermittent Arm Lift
            for k in FullTwistShoulder:
                # Lift Elbow at each intermittent Arm movement
                for l in FullLiftElbow:
                    # Twist Lower arm at each intermittent Elbow position
                    for m in FullTwistElbow:
                        print(FullTwistElbow, m)
                        # Lift Hand at each intermittent Arm and Lower arm position
                        for n in FullLiftWrist:
                            # set new rotation angles
                            print("I'm setting the rotation")
                            PoseHand.rotation_euler = (math.radians(n), math.radians(0), math.radians(0))
                            PoseUpperArm.rotation_euler = (math.radians(0), math.radians(k), math.radians(j)) 
                            PoseShoulderRotation.rotation_euler = (math.radians(0), math.radians(i), math.radians(0))
                            PoseLowerArm.rotation_euler = (math.radians(l), math.radians(m), math.radians(0))
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
                            print (i, j, k, l, m, n)
                           
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
                                counter = counter + 1
    bpy.context.scene.frame_end = counter
    bpy.ops.anim.change_frame(frame = 0)


def main():
    startcounter = 0
    print("Starting...")
	# insert start position at frame 0
    bpy.context.scene.frame_start = 0
    setRestPosition()
    setRangesOfMotion()
    #Clear out the actions from any previous run
    bpy.context.object.animation_data.action = None
    bpy.ops.action.new() 
    rotateAllExtents(startcounter)
    # play the animation we just built
    bpy.ops.screen.animation_play()

main()