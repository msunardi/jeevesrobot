#!/bin/bash
## date 2015.03.15

:<<OpenPtrackInstall2_sh_Description
  1. This is script 2 of 3. This script is run after its predecessor
     has been executed successfully, open_ptrack_install_1.sh
  2. If errors during "catkin_make" command (below): open new terminal 
     window and run this script second time. 
     If still errors, reboot and run it again.
OpenPtrackInstall2_sh_Description

currentyExecutingCommand=''


## ============== Find Catkin Workspace directory: ======================
i=0
array=()
echo >&2 'Searching for catkin workspace directory ....'
for FILE in $(find / -name '.catkin_workspace' 2>/dev/null); do
	CAT_WS="${FILE%.[^.]*}"
  array+=("${CAT_WS%/}")
done

if [ -z "$CAT_WS" ]; then
  echo >&2 'catkin workspace directory not found. Exiting installation script'
  exit
fi 

echo >&2 'Found following catkin workspace(s): '
for elem in "${array[@]}"
do :
  echo $i: $elem
  i=$((i+1))
done

echo -e >&2 'Which catkin workspace do you want to install to (enter index number): \c '
read user_input

if [ "$user_input" -lt 0 -o  "$user_input" -ge "$i" ]; then
  echo >&2 'Incorrect input: ' $user_input
  echo >&2 'Installation aborted.'
  exit
fi

echo >&2 'You chose: ' ${array[$user_input]}
echo -e >&2 'If this is correct, enter Y or y for yes. Else, enter N for no: \c'
read user_input2
if [ "$user_input2" = "y" -o "$user_input2" = "Y" ]; then
   echo >&2 'You entered Y'
else
   echo >&2 'You did not enter "Y" or "y" for "yes". Installatoin aborted.'
   exit
fi
MY_CATKIN_WS_DIR=${array[$user_input]}
echo >&2 'MY_CATKIN_WS_DIR: ' $MY_CATKIN_WS_DIR

  ## these are not needed, only for info:
  ## HOME_DIR_NAME="${CAT_WS%/[^/]*}"
  ## FILENAME="${CAT_WS:${#DIRNAME} + 1}"
	## EXT="${FILE##*\.}"
## ====== End of looking for catkin workspace directory. ===================





source /opt/ros/indigo/setup.bash
source $MY_CATKIN_WS_DIR/devel/setup.bash


cd $MY_CATKIN_WS_DIR

:<<IF_ERRORS_ON_CATKIN_MAKE
  If errors during "catkin_make": open new terminal window and run
  this script second time. If still errors, reboot and run it again.
IF_ERRORS_ON_CATKIN_MAKE


echo >&2 'executing catkin_make --force-cmake'
catkin_make --force-cmake



