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

## Following scripts are from https://github.com/deepguider/dg_cart_ros

## Record rosbag data and visualize it. Bag files will be saved at home directory
roslaunch dg_cart_ros dg_record_sensor.launch

## No record, no visualization. Just bypass sensor data to roscore
#roslaunch dg_cart_ros dg_run_sensor.launch

## No record. Visualiztion.
#roslaunch dg_cart_ros dg_show_sensor.launch

