#!/bin/bash
source /opt/ros/melodic/setup.bash  # It is necessary to run catkin_make

if [ ! -n "$VIRTUAL_ENV" ];then
	echo "You need to run $0 after entering virtualenv"
	exit 0
fi

catkin_make  # It's necessary to makedevel directory
source devel/setup.bash

## Start roscore
pid=`pgrep -f roscore`
if [ ! -n "${pid}" ];then  # If process is not running.
    gnome-terminal --tab -- roscore
    sleep 2s    # wait until roscore is ready
fi

## Run theta360z1 crop publish
pid=`pgrep -f "python2 crop360cam_python27.py"`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
gnome-terminal --tab --title="theta360z1_crop_pub" -- bash -c 'cd ~/catkin_ws/src/dg_cart_ros/src/theta360z1/publish && python2 crop360cam_python27.py'

## Stop rosbag play
pid=`pgrep -f rosbag`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi

## Start dg_simple_ros package (working directory: devel/lib/dg_simple_ros/)
pid=`pgrep -f dg_simple_ros`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi

CWD=`pwd`
LIBDIR="devel/lib/dg_simple_ros"

ln -sf ${CWD}/dg_ros.yml ${LIBDIR}/.
roslaunch dg_simple_ros dg_simple_ros.launch
