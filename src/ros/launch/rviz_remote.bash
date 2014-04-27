#!/usr/bin/env bash

source ./client_hostname_exports.bash
rosrun rviz rviz &
rosrun rqt_console rqt_console
