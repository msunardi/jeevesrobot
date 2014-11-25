#!/bin/bash



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

# If errors on running "catkin_make",
# then open new terminal and re-run.
# If still error, reboot and run catkin_make:
catkin_make --force-cmake

cd ~/Downloads
git clone https://github.com/avin2/SensorKinect
cd SensorKinect/Bin
tar -xjf SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2
cd Sensor-Bin-Linux-x64-v5.1.2.1
sudo ./install.sh


