#!/bin/bash

echo >&2 'installing Avin2 SensorKinect dirver'
currentlyExecutingCommand='git clone https://github.com/avin2/SensorKinect'
cd ~/Downloads
git clone https://github.com/avin2/SensorKinect
cd SensorKinect/Bin
tar -xjf SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2
cd Sensor-Bin-Linux-x64-v5.1.2.1
currentlyExecutingCommand='./install.sh'
echo >&2 'runnint: ./install.sh of SensorKinect, Avin2 driver installation'
sudo ./install.sh
