#!/bin/bash

if [ $# -ne 1 ]
then
    echo "Usage: set_current_pose_to_waypoint.sh waypoint_name"
    exit
fi

rosservice call /waypoint_manager/set_current_pose_to_waypoint $1