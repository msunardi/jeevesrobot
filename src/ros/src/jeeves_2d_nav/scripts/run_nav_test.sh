#!/bin/bash

rostopic pub --once /nav_test/cmd  std_msgs/String "RUN"
