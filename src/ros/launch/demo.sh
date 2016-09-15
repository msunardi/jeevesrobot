#!/usr/bin/env bash

JEEVES_HOME=$HOME/src/roboticsclub-mcecsbot/src/ros
JEEVES_LAUNCH=$JEEVES_HOME/launch
JEEVES_WEB_UI=$JEEVES_HOME/src/web_ui
function start {
    echo "Starting demo ..."
    start_launch
    start_facial_recognition
    start_web_ui
}

function start_launch {
    LAUNCH_FILE='summer2016.launch'
    echo "Starting launch file: $LAUNCH_FILE ..."
    cd $JEEVES_LAUNCH
    roslaunch summer2016.launch &
}

function start_facial_recognition {
    echo "Starting facial recognition node ..."
    cd $JEEVES_HOME
    rosrun facial_recognition facial_recognition_node &
}

function start_web_ui {
    echo "Starting web UI ... at 192.168.1.2:8000"
    cd $JEEVES_WEB_UI
    ./web_ui start &
}

function stop_web_ui {
    cd $JEEVES_WEB_UI
    ./web_ui stop
}

function stop_all {
    echo "Stopping everything ..."
    pkill -f 'summer2016.launch'
    pkill -f 'facial_recognition_node'
    pkill -f 'pocketsphinx'
    stop_web_ui
    echo "Done."
}   

case "$1" in
    start) start &> log/demo.log;;
    stop) stop_all;;
    restart) stop_all; start &;;
    *) echo "usage $0 start|stop|restart" >&2
        exit 1
        ;;
esac
    
