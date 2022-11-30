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

function set_robot_map_idx(){  
    # You can modify the robot_map_index parameter in dg_ros.yml
    #                          0             1                       2          3             
    local index=${1} #0:"Bucheon_KETI", 1:"Bucheon_Robot", 2:"COEX_KETI", 3:"COEX_Robot"
    local param="^robot_map_index"
    local param_dst="robot_map_index"
    sed -i "s|${param}: [0-9]|${param_dst}: ${index}|g" dg_ros.yml
}

## Run theta360z1 crop publish
pid=`pgrep -f "python2 crop360cam_python27.py"`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
CWD=`pwd`
gnome-terminal --tab --title="theta360z1_crop_pub" -- bash -c "cd ${CWD}/src/dg_cart_ros/src/theta360z1/publish && python2 crop360cam_python27.py"

#set_site_idx 2; set_topic_idx 2; set_robot_map_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 150 -r 1 ./recordings/Rosbag_subgoal/20221108_bucheon.bag
#set_site_idx 1; set_topic_idx 2; set_robot_map_idx 3; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 1000 -r 1 ./recordings/Rosbag_subgoal/20221122_COEX.bag
set_site_idx 2; set_topic_idx 2; set_robot_map_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 50 -r 1 ./recordings/Rosbag_subgoal/20221108_bucheon.bag
#set_site_idx 1; set_topic_idx 2; set_robot_map_idx 3; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 150 -r 1 ./recordings/Rosbag_subgoal/20221122_COEX.bag

## Start dg_simple_ros package (working directory: devel/lib/dg_simple_ros/)
LIBDIR="devel/lib/dg_simple_ros"

cp -rf data* font logo_data model ${LIBDIR}/.
ln -sf ${CWD}/recordings ${LIBDIR}/.
ln -sf ${CWD}/dg_ros.yml ${LIBDIR}/.

roslaunch dg_simple_ros dg_simple_robot.launch
