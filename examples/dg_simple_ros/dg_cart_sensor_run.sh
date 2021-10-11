#!/bin/bash

source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash

## Start roscore
pid=`pgrep roscore`
if [ ! -n "${pid}" ];then  # If process is not running.
    gnome-terminal -- roscore
    sleep 2s    # wait until roscore is ready
fi

## Start recording cart sensor
pid=`pgrep roslaunch`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi

roslaunch dg_cart_ros dg_run_sensor.launch
