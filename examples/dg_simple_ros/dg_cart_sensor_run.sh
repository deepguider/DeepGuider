#!/bin/bash

if [ -n "$VIRTUAL_ENV" ];then
	echo "You need to run $0 after deactivate virtual environment"
	echo "Run following"
	echo "deactivate; $0"
	exit 0
fi

source /opt/ros/melodic/setup.bash  # It is necessary to run catkin_make
catkin_make
source devel/setup.bash

## Start roscore
pid=`pgrep roscore`
if [ ! -n "${pid}" ];then  # If process is not running.
    gnome-terminal --tab -- roscore
    sleep 2s    # wait until roscore is ready
fi

# Run door detect in python 3 environment
pid=`pgrep -f door_detect_rospublisher`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
gnome-terminal --tab --title="door_detect" -- bash -c 'source ~/.virtualenvs/dg_venv3.6/bin/activate && cd ~/catkin_ws/src/dg_cart_ros/src/door_detect && python door_detect_rospublisher.py'

## Run theta360z1 publish
pid=`pgrep main_ros_python27`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
gnome-terminal --tab --title="theta360z1_pub" -- bash -c 'source deactivate && cd ~/catkin_ws/src/dg_cart_ros/src/theta360z1/publish && python main_ros_python27.py'

# Run andro2linux_gps publish
pid=`pgrep -f andro2linux_gps_rospublisher`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
gnome-terminal --tab --title="andro2linux_gps_pub" -- bash -c 'source deactivate && cd ~/catkin_ws/src/dg_cart_ros/src/andro2linux_gps/publish && python andro2linux_gps_rospublisher.py'

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

