#!/bin/bash
# source this file before running roscore or a roslaunch file on jeeves
# if you want remote clients to connect to the ros master on jeeves.

export ROS_IP=192.168.1.2
export ROS_HOSTNAME=jeeves
export ROS_MASTER_URI=http://192.168.1.2:11311
