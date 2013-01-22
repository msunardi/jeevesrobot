# Maisee Brown
# 10/29/2012
# TwistElbow.py (Arm Movements Collection 5/6)
# This script is one of six that are intended to show the basic degrees of freedom 
# that a robotic arm I am simulating in blender will be able to perform once built.

from armInclude import *

def twistElbow(counter):
# Set mode to pose mode
    bpy.ops.object.mode_set(mode='POSE')
    #start frame counter at 0
  #  counter = 1
    bpy.ops.anim.change_frame(frame = counter)
    # move the hand to each angle while updating the frame numbers
    for k in range(0, ElbowTwistMin-ElbowTwistStep, -ElbowTwistStep):
        # set new rotation angles
        PoseLowerArm.rotation_euler = (math.radians(0), math.radians(k), math.radians(0))
        # insert key frame so we can mark this position in our animation
        PoseLowerArm.keyframe_insert(data_path="rotation_euler")
        # increment counter so the animation progresses in time
        counter = counter + counterIncrement
        bpy.ops.anim.change_frame(frame = counter)
    for j in range(ElbowTwistMin, ElbowTwistMax+ElbowTwistStep, ElbowTwistStep):
        # set new rotation angles
        PoseLowerArm.rotation_euler = (math.radians(0), math.radians(j), math.radians(0))
        #insert key frame so we can mark this position in our animation
        PoseLowerArm.keyframe_insert(data_path="rotation_euler")
        # increment counter so the animation progresses in time
        counter = counter + counterIncrement
        # set frame to new counter value
        bpy.ops.anim.change_frame(frame = counter)
    for j in range(ElbowTwistMax, 0-ElbowTwistStep, -ElbowTwistStep):
        # set new rotation angles
        PoseLowerArm.rotation_euler = (math.radians(0), math.radians(j), math.radians(0))
        #insert key frame so we can mark this position in our animation
        PoseLowerArm.keyframe_insert(data_path="rotation_euler")
        # increment counter so the animation progresses in time
        counter = counter + counterIncrement
        # set frame to new counter value
        bpy.ops.anim.change_frame(frame = counter)
    # when finished set the frame to frame zero so that when we animate it starts from the beginning
    bpy.context.scene.frame_end = counter + counterIncrement
    bpy.ops.anim.change_frame(frame = 0)

def main():
    print("Starting...")
	# This is a counter used to string the 6 motions in some order later after the initial scripts are running
    startcounter = 1
    #Clear out the actions from any previous run
    bpy.context.object.animation_data.action = None
    bpy.context.scene.frame_start = 0
	# Set the arm into the rest position incase it was left by a previous script in some other position
    setRestPosition()
    #Create new action
    bpy.ops.action.new() 
	#run the swing shoulder function
    twistElbow(startcounter)
    # play the animation we just built
    bpy.ops.screen.animation_play()

main()