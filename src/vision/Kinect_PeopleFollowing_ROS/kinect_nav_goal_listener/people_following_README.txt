
2015/03/09

1. Created a single proper ROS script, named follow_people.launch.
This script can be run from remote machine.
Previous script was people_following_non_remotely.sh, which was not proper (non-ROS script).
2. Task for the future: need to consolidate and streamline Kinect People Following packages (nodes), as follows. Currently, two custom packages (nodes) in catkin "src" directory are used to run People Follower: detector_markers_array_listener and kinect_nave_goal_listener. Existence of such multiple packages in catkin "src" directory, to achieve a single goal of people following, clutters the catin "src" directory. These packages need to be placed in one single directory (package) named, for example, kinect_people_follower, or such, in catkin "src" directory. IN this "kinect_people_follower" directory (package), there should be "nodes" subdirectory where the two above named nodes (packages) should reside.
This will be done after people follower packages work 100% as desired. 
Currently, Jeeves weers a bit to the right after achieveing the target (detected person). 

