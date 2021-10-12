#!/bin/bash

if [ ! -n "$VIRTUAL_ENV" ];then
	echo "You need to run $0 after entering virtualenv"
	exit 0
fi

source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash

## Start roscore
pid=`pgrep roscore`
if [ ! -n "${pid}" ];then  # If process is not running.
    gnome-terminal -- roscore
    sleep 2s    # wait until roscore is ready
fi

## Start rosbag play
pid=`pgrep rosbag`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
gnome-terminal -- rosbag play --pause -s 500 -r 10 ./recordings/2020-02-19-15-01-53.bag
echo "Press [Space] to start rosbag play on the gnome terminal"

## Start dg_ocr
#pid=`pgrep dg_ocr`
#if [ -n "${pid}" ];then  # If process is running.
#    kill -9 ${pid}
#fi
#gnome-terminal -- rosrun dg_simple_ros dg_ocr # run ocr_ros first in another console

## Start dg_simple_ros
pid=`pgrep dg_simple_ros`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
rosrun dg_simple_ros dg_simple_ros            # run dg_simple_ros
