#!/bin/bash

abort()
{
    echo >&2 '
***************
*** ABORTED ***
***************
'
    exit 1
}
trap 'abort' 0
set -e    # aborts script on any error


for FILE in $(find / -name '.catkin_workspace' 2>/dev/null); do
	CAT_WS="${FILE%.[^.]*}"
        MY_CATKIN_WS_DIR="${CAT_WS%/}"
        HOME_DIR_NAME="${CAT_WS%/[^/]*}"
        FILENAME="${CAT_WS:${#DIRNAME} + 1}"
	EXT="${FILE##*\.}"
done
# echo >&2 'MY_CATKIN_WS_DIR: ' $CATKIN_WS_DIR_NAME

source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash


ROS_PACKAGES="python-rosinstall ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-cmake-modules ros-$ROS_DISTRO-freenect-stack ros-$ROS_DISTRO-openni-launch ros-$ROS_DISTRO-camera-info-manager-py"

sudo apt-get install $ROS_PACKAGES

echo "export KINECT_DRIVER=freenect" >> ~/.bashrc
echo "export LC_ALL=C" >> ~/.bashrc
export KINECT_DRIVER=freenect
export LC_ALL=C


source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash

cd $MY_CATKIN_WS_DIR/src
git clone https://github.com/OpenPTrack/open_ptrack.git
cd open_ptrack/scripts
sudo chmod +x *.sh

./ceres_install_trusty.sh

cd $MY_CATKIN_WS_DIR/src
git clone https://github.com/iaslab-unipd/calibration_toolkit
cd calibration_toolkit
git fetch origin --tags
git checkout tags/v0.2

# Update to v0.4 of libfreenect driver for Kinect:
cd ~
mkdir libfreenect
cd libfreenect
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
git checkout tags/v0.4.0
mkdir build
cd build
cmake -L ..
make

sudo mv ~/libfreenect/libfreenect/build/lib/fakenect/* /opt/ros/indigo/lib/fakenect/
sudo mv ~/libfreenect/libfreenect/build/lib/libfreenect* /opt/ros/indigo/lib/
sudo mv ~/libfreenect/libfreenect/build/src/libfreenect.pc /opt/ros/indigo/lib/pkgconfig/
sudo mv ~/libfreenect/libfreenect/include/libfreenect.h /opt/ros/indigo/include/libfreenect/libfreenect.h
sudo mv ~/libfreenect/libfreenect/include/libfreenect_registration.h /opt/ros/indigo/include/libfreenect/libfreenect-registration.h
sudo mv ~/libfreenect/libfreenect/wrappers/cpp/libfreenect.hpp /opt/ros/indigo/include/libfreenect/libfreenect.hpp
sudo mv ~/libfreenect/libfreenect/wrappers/c_sync/libfreenect_sync.h /opt/ros/indigo/include/libfreenect/libfreenect_sync.h
cd $MY_CATKIN_WS_DIR/src
git clone https://github.com/ros-drivers/freenect_stack.git
sudo rm -R ~/libfreenect

if [ -d "~/Downloads" ]
then 
	echo ""
else
	mkdir -p ~/Downloads
fi 
cd $MY_CATKIN_WS_DIR/src/open_ptrack/scripts
./mesa_install.sh

source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash
cd $MY_CATKIN_WS_DIR
catkin_make --pkg calibration_msgs
catkin_make --pkg opt_msgs



trap : 0
echo >&2 '
**********************************
*** DONE: there were no errors *** 
**********************************
'
