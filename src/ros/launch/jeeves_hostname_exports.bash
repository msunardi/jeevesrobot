#!/bin/bash
# source this file before running roscore or a roslaunch file on jeeves
# if you want remote clients to connect to the ros master on jeeves.

export ROS_IP=jeeves.dnsdynamic.com
export ROS_HOSTNAME=jeeves.dnsdynamic.com
export ROS_MASTER_URI=http://jeeves.dnsdynamic.com:11311
