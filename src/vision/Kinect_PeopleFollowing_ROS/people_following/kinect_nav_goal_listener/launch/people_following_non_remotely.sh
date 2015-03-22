#!/bin/bash
## 2015.01.18
## Launches kinect people tracking nodes and people following node

FILE='/home/mcecsbot/src/roboticsclub-mcecsbot/src/ros/src/open_ptrack/tracking/launch/detection_and_tracking_no_gui.launch'
if [ ! -f "$FILE" ]; then
  echo 'file' "$FILE" 'not found'
  echo 'copy the file detection_and_tracking.launch, rename it to detection_and_tracking_no_gui.launch'
  echo 'and remove the gui launching statement (visualization), udp messaging statement, and other statements,'
  echo 'but leave only first two statements, People detection and tracking, and run this script again.'
  exit
fi

gnome-terminal -e '/opt/ros/indigo/bin/roslaunch tracking detection_and_tracking_no_gui.launch' &
gnome-terminal -e '/opt/ros/indigo/bin/rosrun detector_markers_array_listener detector_markers_array_listener' &
gnome-terminal -e '/opt/ros/indigo/bin/rosrun kinect_nav_goal_listener kinect_nav_goal_listener' &



## ---------------------other stuff, for reference and examples:-----------------------
# xterm -hold roscore &
# sleep 4
# gnome-terminal -e '/opt/ros/indigo/bin/roslaunch tracking detection_and_tracking.launch' &

