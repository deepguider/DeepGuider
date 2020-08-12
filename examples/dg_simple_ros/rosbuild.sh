#!/bin/bash

## Set path of deepguider and ros workspace
#ROSDIR="/home/ccsmm/dg_ws" # path of your own ROS workspace. This script will create it when it does not exist.

if [ -z $1 ]; then
	echo "Usage : $0 /home/your_home/your_ros_ws_path"
	echo "      If this path does not exist, then it will be created automatically"
	echo "      If this path exists, then it will be overwrited"
	echo "      Use Absolute path rather than relative one."
	exit -1
fi

ROSDIR=$1 # path of your own ROS workspace. This script will create it when it does not exist.

sed -e "s|/home/dg/dg_ws|$ROSDIR|g" rosbuild_template.txt > .my_rosbuild.sh
chmod +x .my_rosbuild.sh
./.my_rosbuild.sh
rm .my_rosbuild.sh
