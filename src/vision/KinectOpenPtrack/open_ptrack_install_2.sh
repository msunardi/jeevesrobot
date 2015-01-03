#!/bin/bash
## date 2015.01.03
:<<OpenPtrackInstall2_sh_Description
  1. This is script 2 of 3. This script is run after its predecessor
     has been executed successfully, open_ptrack_install_1.sh
  2. If errors during "catkin_make" command (below): open new terminal 
     window and run this script second time. 
     If still errors, reboot and run it again.
OpenPtrackInstall2_sh_Description

currentyExecutingCommand=''
echo >&2 'searching for catkin workspace directory ...'
for FILE in $(find / -name '.catkin_workspace' 2>/dev/null); do
	CAT_WS="${FILE%.[^.]*}"
        MY_CATKIN_WS_DIR="${CAT_WS%/}"
        HOME_DIR_NAME="${CAT_WS%/[^/]*}"
        FILENAME="${CAT_WS:${#DIRNAME} + 1}"
	EXT="${FILE##*\.}"
done
if ["$CAT_WS" == ""]; then
  echo >&2 'catkin workspace directory not found. Exiting installation script'
  exit
else 
  echo >&2 'Found your catkin workspace directory: ' $CAT_WS
fi 

source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash


cd $MY_CATKIN_WS_DIR

:<<IF_ERRORS_ON_CATKIN_MAKE
  If errors during "catkin_make": open new terminal window and run
  this script second time. If still errors, reboot and run it again.
IF_ERRORS_ON_CATKIN_MAKE


echo >&2 'executing catkin_make --force-cmake'
catkin_make --force-cmake



