#!/bin/bash

if [ $# -ne 1 ]
then
    echo "Usage: save_current_pose.sh waypoint_name"
    exit
fi

rosservice call /waypoint_manager/save_current_pose $1
