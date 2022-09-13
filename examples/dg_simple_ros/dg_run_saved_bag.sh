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

## Start rosbag play
pid=`pgrep -f "rosbag play"`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
# --pause: pause rosbag play until spacebar key pressed
# -d n: delay rosbag play n seconds
# -s n: skip first n seconds
# -r k: replay rosbag k times faster than normal speed

function set_site_idx(){
    # You can modify the site_index parameter in dg_ros.yml
    #                     0        1        2          3             4            5                   6
    local index=${1} # ["ETRI", "COEX", "Bucheon", "TEST_ETRI", "TEST_COEX", "TEST_ETRI_INDOOR", "ETRI_EXTENDED"]
    local param="^site_index"
    local param_dst="site_index"
    sed -i "s|${param}: [0-9]|${param_dst}: ${index}|g" dg_ros.yml
}

function set_topic_idx(){
    # You can modify the topic_name_index parameter in dg_ros.yml
    #                         0                 1             2
    local index=${1} # ["ETRI_CART_VER2_ANDROIDGPS", "ETRI_CART_VER2", "KETI_ROBOT", "ETRI_CART_VER1"]
    local param="topic_name_index"
    local param_dst=${param}
    sed -i "s|${param}: [0-9]|${param_dst}: ${index}|g" dg_ros.yml
}

## Run theta360z1 crop publish
pid=`pgrep -f "python2 crop360cam_python27.py"`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
gnome-terminal --tab --title="theta360z1_crop_pub" -- bash -c 'cd ~/catkin_ws/src/dg_cart_ros/src/theta360z1/publish && python2 crop360cam_python27.py'

#set_site_idx 6; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/20220324_ETRI_and_Doryong.bag /dg_odometry/pose:=/dg_odometry_record/pose
#set_site_idx 4; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/2021-10-05-13-09-40.bag /dg_odometry/pose:=/dg_odometry_record/pose
#set_site_idx 4; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/220418_coex_out_to_indoor/_2022-04-18-14-08-03.bag /dg_odometry/pose:=/dg_odometry_record/pose
#set_site_idx 7; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/220418_coex_out_to_indoor/_2022-04-18-13-20-44.bag /dg_odometry/pose:=/dg_odometry_record/pose  # no gps test with vps at COEX
set_site_idx 7; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 1 ./recordings/220829_coex/_2022-08-29-11-52-58.bag /dg_odometry/pose:=/dg_odometry_record/pose # coex indoor
#set_site_idx 4; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/coex.bag /dg_odometry/pose:=/dg_odometry_record/pose
#set_site_idx 3; set_topic_idx 2; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 600 -r 10 ./recordings/etri.bag /dg_odometry/pose:=/dg_odometry_record/pose
#set_site_idx 5; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 0 -r 1 ./recordings/indoor.bag /dg_odometry/pose:=/dg_odometry_record/pose

## Start dg_simple_ros package (working directory: devel/lib/dg_simple_ros/)
CWD=`pwd`
LIBDIR="devel/lib/dg_simple_ros"

cp -rf data* font logo_data model recordings ${LIBDIR}/.
ln -sf ${CWD}/dg_ros.yml ${LIBDIR}/.

roslaunch dg_simple_ros dg_simple_ros.launch
