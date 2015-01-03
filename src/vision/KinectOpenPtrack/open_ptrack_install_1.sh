#!/bin/bash
## date:       2015.01.03
:<<OpenPtrackInstall1_sh_Description
  Installs open_ptrack, aka People Detection and Tracking Package,
  and its dependencies. This is script 1 of 3. 
  After running this script, run next script, named open_ptrack_install_2.sh
  Assumptions:
    1. ROS Indigo, Full Desktop Version has been installed.
    2. Catkin Workspace has been initialized.
OpenPtrackInstall1_sh_Description

currentlyExecutingCommand=""

abort()
{
    echo >&2 '
***************
*** ABORTED ***
***************
'
echo >&2 'error was in executing of this: ' $currentlyExecutingCommand
    exit 1
}
trap 'abort' 0
set -e    # aborts script on any error

echo >&2 'Searching for catking workspace directory ....'
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

ROS_PACKAGES="python-rosinstall ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-cmake-modules ros-$ROS_DISTRO-freenect-stack ros-$ROS_DISTRO-openni-launch ros-$ROS_DISTRO-camera-info-manager-py"

echo >&2 'installing: ' $ROS_PACKAGES
currentlyExecutingCommand="sudo apt-get install ROS_PACKAGES"
sudo apt-get install $ROS_PACKAGES

echo >&2 'exporing KINECT_DRIVER=freenect and LC_ALL=C to .bashrc'
echo >&2 "export KINECT_DRIVER=freenect" >> ~/.bashrc
echo >&2 "export LC_ALL=C" >> ~/.bashrc
export KINECT_DRIVER=freenect
export LC_ALL=C

echo >&2 "source $MY_CATKIN_WS_DIR/devel/setup.bash" >> ~/.bashrc
source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash

cd $MY_CATKIN_WS_DIR/src

echo >&2 'cloning: git clone https://github.com/OpenPTrack/open_ptrack.git'
currentlyExecutingCommand='git clone https://github.com/OpenPTrack/open_ptrack.git'
git clone https://github.com/OpenPTrack/open_ptrack.git
cd open_ptrack/scripts
sudo chmod +x *.sh

echo >&2 'executing ./ceres_install_trusty.sh'
currentlyExecutingCommand='./ceres_install_trusty.sh'
./ceres_install_trusty.sh

cd $MY_CATKIN_WS_DIR/src

echo >&2 'cloning: git clone https://github.com/iaslab-unipd/calibration_toolkit'
currentlyExecutingCommand='git clone https://github.com/iaslab-unipd/calibration_toolkit'
git clone https://github.com/iaslab-unipd/calibration_toolkit
cd calibration_toolkit

echo >&2 'running git fetch origin --tags, and git checkout tags/v0.2'
git fetch origin --tags
git checkout tags/v0.2

## Use either ros-indigo-freenect Kinect driver, commented as "ROS_S_LIBFREENECT"
## or "leebfreenect below, "original" open_ptrack's libfreenect installation 
## they will be overwriten by Avin2 drivers, anyway, but they might be
## required to run open_ptrack:

:<<ROS_S_LIBFREENECT
## or "leebfreenect below, "original" open_ptrack's libfreenect installation 
echo >&2 'Installing libfreenect driver for Kinect'
echo >&2 'running sudo apt-get install libfreenect-dev'
currentlyExecutingCommand='apt-get install libfreenect-dev'
sudo apt-get install libfreenect-dev

echo >&2 'running apt-get install ros-indigo-freenect-launch'
currentlyExecutingCommand='apt-get install ros-indigo-freenect-launch'
sudo apt-get install ros-indigo-freenect-launch
ROS_S_LIBFREENECT



## open_ptrack's own libfreenect Kinect Drivers:
## Use these drivers. Else, there might be some "roi_msgs" error.
## open_ptrack's own libfreenect, use this one.
## It will be overwritten by Avin2 drivers, but it is still required for
## installation.
## Update to v0.4 of libfreenect driver for Kinect:
cd ~
mkdir libfreenect
cd libfreenect
echo >&2 'running: git clone https://github.com/OpenKinect/libfreenect.git'
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
git checkout tags/v0.4.0
mkdir build
cd build
currentlyExecutingCommand='cmake -L of libfreenect installation'
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

cd $MY_CATKIN_WS_DIR/src/open_ptrack/scripts
echo >&2 'installing Mesa SwissRanger driver ....'
currentlyExecutingCommand='sudo dpkg -i libmesasr-dev-1.0.14-748'
# Install Mesa SwissRanger driver:
cd ~/Downloads
if [ "$(uname -m)" = "x86_64" ]; then
wget -U "Mozilla" http://www.mesa-imaging.ch/customer/driver/libmesasr-dev-1.0.14-748.amd64.deb
sudo dpkg -i libmesasr-dev-1.0.14-748.amd64.deb 
elif [ "$(uname -m)" = "i686" ]; then
wget -U "Mozilla" http://www.mesa-imaging.ch/customer/driver/libmesasr-dev-1.0.14-747.i386.deb
sudo dpkg -i libmesasr-dev-1.0.14-747.i386.deb
# http://downloads.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.14-747.i386.deb
else
  echo >&2 'Could not determine the bus width architecture of your machine'
  echo >&2 'Manualy install SwissRanger driver'
  exit
fi

echo >&2 "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo >&2 "source $MY_CATKIN_WS_DIR/devel/setup.bash" >> ~/.bashrc

source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash
cd $MY_CATKIN_WS_DIR

echo >&2 'running catkin_make --pkg calibration_msgs'
currentlyExecutingCommand='catkin_make --pkg calibration_msgs'
catkin_make --pkg calibration_msgs

echo >&2 'running catkin_make --pkg opt_msg'
echo >&2 'If there is ROI_msgs error here, re-run catkin_make --pkg opt_msg'
currentlyExecutingCommand='catkin_make --pkg opt_msgs'
catkin_make --pkg opt_msgs


trap : 0
echo >&2 '
************
*** DONE ***
************
'
