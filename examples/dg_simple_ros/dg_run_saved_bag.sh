#!/bin/bash
source /opt/ros/melodic/setup.bash  # It is necessary to run catkin_make

if [ ! -n "$VIRTUAL_ENV" ];then
	echo "You need to run $0 after entering virtualenv"
	exit 0
fi

catkin_make -DCMAKE_BUILD_TYPE=Release # It's necessary to makedevel directory
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
    #                                 0                       1             2                3                  4        
    local index=${1} # ["ETRI_CART_VER2_ANDROIDGPS", "ETRI_CART_VER2", "KETI_ROBOT", "ETRI_CART_VER1", "ETRI_CART_VER2_RTKGPS"]
    local param="topic_name_index"
    local param_dst=${param}
    sed -i "s|${param}: [0-9]|${param_dst}: ${index}|g" dg_ros.yml
}

## Run theta360z1 crop publish
pid=`pgrep -f "python2 crop360cam_python27.py"`
if [ -n "${pid}" ];then  # If process is running.
    kill -9 ${pid}
fi
CWD=`pwd`
gnome-terminal --tab --title="theta360z1_crop_pub" -- bash -c "cd ${CWD}/src/dg_cart_ros/src/theta360z1/publish && python2 crop360cam_python27.py"

## Bucheon Final Demo
set_site_idx 2; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 60 -r 1 ./recordings/_2022-11-17-13-49-19_final_demo_keti2nong.bag

## COEX Final Demo

## RTKGPS test
#set_site_idx 6; set_topic_idx 4; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 0 -r 10 ./recordings/with_rtk_gps/_2023-03-15-21-05-25.bag

## Test
#set_site_idx 6; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/20220324_ETRI_and_Doryong.bag
#set_site_idx 4; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/2021-10-05-13-09-40.bag
#set_site_idx 4; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/220418_coex_out_to_indoor/_2022-04-18-14-08-03.bag
#set_site_idx 7; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/220418_coex_out_to_indoor/_2022-04-18-13-20-44.bag
#set_site_idx 7; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 1 ./recordings/220829_coex/_2022-08-29-11-52-58.bag
#set_site_idx 7; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 1 ./recordings/220829_coex/_2022-08-29-13-23-17.bag
#set_site_idx 2; set_topic_idx 0; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 1 ./recordings/_2022-11-08-16-05-30_keti2nonghyup_notbad.bag
#set_site_idx 4; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 0 -r 10 ./recordings/coex.bag
#set_site_idx 3; set_topic_idx 2; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 10 -s 600 -r 10 ./recordings/etri.bag
#set_site_idx 5; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 0 -r 1 ./recordings/indoor.bag
#set_site_idx 2; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 0 -r 1 ./recordings/_2022-10-06-14-40-21.bag
#set_site_idx 2; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 0 -r 1 ./recordings/_2022-10-06-13-49-15.bag
#set_site_idx 2; set_topic_idx 1; gnome-terminal --tab --title="PlayingRosbag" -- rosbag play -d 5 -s 0 -r 1 ./recordings/_2022-10-06-14-18-35.bag

## Start dg_simple_ros package (working directory: devel/lib/dg_simple_ros/)
LIBDIR="devel/lib/dg_simple_ros"

cp -rf data* font logo_data model ${LIBDIR}/.
ln -sf ${CWD}/recordings ${LIBDIR}/.
ln -sf ${CWD}/dg_ros.yml ${LIBDIR}/.

roslaunch dg_simple_ros dg_simple_ros_test.launch
