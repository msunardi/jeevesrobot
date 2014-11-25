#!/bin/bash
:<<OpenPtrackInstall2_sh_Description
  1. This is script 2 of 2. This script is run after its predecessor
     has been executed successfully, open_ptrack_install_2.sh
  2. If errors during "catkin_make" command (below): open new terminal 
     window and run this script second time. 
     If still errors, reboot and run it again.
OpenPtrackInstall2_sh_Description


for FILE in $(find / -name '.catkin_workspace' 2>/dev/null); do
	CAT_WS="${FILE%.[^.]*}"
        MY_CATKIN_WS_DIR="${CAT_WS%/}"
        HOME_DIR_NAME="${CAT_WS%/[^/]*}"
        FILENAME="${CAT_WS:${#DIRNAME} + 1}"
	EXT="${FILE##*\.}"
done


source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash
export KINECT_DRIVER=freenect
export LC_ALL=C


cd $MY_CATKIN_WS_DIR

:<<IF_ERRORS_ON_CATKIN_MAKE
  If errors during "catkin_make": open new terminal window and run
  this script second time. If still errors, reboot and run it again.
IF_ERRORS_ON_CATKIN_MAKE

catkin_make --force-cmake


cd ~/Downloads
git clone https://github.com/avin2/SensorKinect
cd SensorKinect/Bin
tar -xjf SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2
cd Sensor-Bin-Linux-x64-v5.1.2.1
sudo ./install.sh


