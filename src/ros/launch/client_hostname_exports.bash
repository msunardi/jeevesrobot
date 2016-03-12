#!/bin/bash
# Edit this file, then source it on any client machine that needs
# communication with the ROS master on jeeves.

# IP of the remote client.
export ROS_IP=192.168.100.134

# Hostname of the remote client
# (or simply the IP, if you don't have name resolution).
export ROS_HOSTNAME=192.168.100.134

# URI of the robot
export ROS_MASTER_URI=http://192.168.1.2:11311
