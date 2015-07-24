#!/bin/bash

if [ $# -ne 1 ]
then
  echo "Usage: "$0 "waypoint_name"
  exit
fi

rosservice call /waypoint_manager/delete_waypoint $1
