# Maisee Brown
# 11/ 5/ 2012
# Wave.py
# This program is intended to show an outline for creating a simulation of a pre-existing armature movement. This movement will be a wave but you can follow the same structure to make other movements if you know the angles of the arm you are trying to achieve.


from armInclude import *
def wave(counter):
    # Set mode to pose mode
    bpy.ops.object.mode_set(mode='POSE')
	
	# set frame to counter to start motion
    bpy.ops.anim.change_frame(frame = counter)
    # set up arrays of angles
    SimSwingShoulder = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    SimLiftShoulder = [0, 0, -30, -60, -60, -60, -60, -60, -60,-60, -60, -60, -30, 0]
    SimTwistShoulder = [0, -20, -40, -40, -40, -40, -40, -40, -40, -40, -40, -40, -20, 0]
    SimLiftElbow = [0, 0, -30, -60, -90, -120, -90, -120, -90, -120, -90, -60, -30, 0]
    SimTwistElbow = [0, 0, 0, 0, 0,0,0,0,0,0,0, 0, 0, 0]
    SimLiftWrist = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	#elementsNeeded = 14
    SimSync = range(0,14)
	
	# for each of the intermediate sets of angles set a key frame
    for i in SimSync:
        # set new rotation angles
        PoseHand.rotation_euler = (math.radians(SimLiftWrist[i]), math.radians(0), math.radians(0))
        PoseUpperArm.rotation_euler = (math.radians(0), math.radians(SimTwistShoulder[i]), math.radians(SimLiftShoulder[i])) 
        PoseShoulderRotation.rotation_euler = (math.radians(0), math.radians(SimSwingShoulder[i]), math.radians(0))
        PoseLowerArm.rotation_euler = (math.radians(SimLiftElbow[i]), math.radians(SimTwistElbow[i]), math.radians(0))
                
        handTailPosition = PoseHand.tail 
        handCenterPosition = PoseHand.center 
        handHeadPosition = PoseHand.head
        lowerHeadPosition = PoseLowerArm.head
        lowerCenterPosition = PoseLowerArm.center
        upperCenterPosition = PoseUpperArm.center
         
		# move the armature by inserting a keyframe
        PoseUpperArm.keyframe_insert(data_path="rotation_euler")
        PoseShoulderRotation.keyframe_insert(data_path="rotation_euler")
        PoseLowerArm.keyframe_insert(data_path="rotation_euler")
        PoseHand.keyframe_insert(data_path="rotation_euler")
        
		# verify that the arm has not hit the body
		# if it has remove the keyframe, otherwise advance to the next position
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
            # set frame to new counter value
            bpy.ops.anim.change_frame(frame = counter)                                                    # increment counter so the animation progresses in time
            print(counter)    
            counter = counter + counterIncrement
    bpy.context.scene.frame_end = counter
	# reset animation so it starts at the beginning
    bpy.ops.anim.change_frame(frame = 0)


def main():
    startcounter = 0
    print("Starting...")
	#start at frame 0
    bpy.context.scene.frame_start = 0
	# insert rest position at frame 0
    setRestPosition()
    bpy.ops.anim.keyframe_insert()
    #Clear out the actions from any previous run
    bpy.context.object.animation_data.action = None
	# Make new action
    bpy.ops.action.new() 
	# Create the motion
    wave(startcounter)
    # play the animation we just built
    bpy.ops.screen.animation_play()

main()