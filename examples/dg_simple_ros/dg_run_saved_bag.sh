#!/bin/bash
source /opt/ros/melodic/setup.bash  # It is necessary to run catkin_make

if [ ! -n "$VIRTUAL_ENV" ];then
	echo "You need to run $0 after entering virtualenv"
	exit 0
fi

catkin_make  # It's necessary to makedevel directory
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
# --pause: pause rosbag play until spacebar key pressed
# -d n: delay rosbag play n seconds
# -s n: skip first n seconds
# -r k: replay rosbag k times faster than normal speed
gnome-terminal -- rosbag play -d 10 -s 0 -r 10 ./recordings/2021-10-05-13-09-40.bag
#gnome-terminal -- rosbag play -d 10 -s 0 -r 10 ./recordings/coex.bag
#gnome-terminal -- rosbag play -d 10 -s 0 -r 10 ./recordings/etri.bag
#gnome-terminal -- rosbag play -d 10 -s 0 -r 1 ./recordings/indoor.bag

## Start dg_simple_ros package (working directory: devel/lib/dg_simple_ros/)
CWD=`pwd`
LIBDIR="devel/lib/dg_simple_ros"

cp -rf data* font logo_data model recordings ${LIBDIR}/.
ln -sf ${CWD}/dg_ros.yml ${LIBDIR}/.

roslaunch dg_simple_ros dg_simple_ros.launch
