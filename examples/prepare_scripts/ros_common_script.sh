#!/bin/bash
#by ccsmm@etri.re.kr

## Usage : insert follwing one line to top of your script.
##         source ros_common_script.sh ## $rosbag_file, $output_dir, $ofdir, and $poses_latlon_compare_file are defined in ros_common_script.sh

## rosbag_file can be a file or a directory. If it is directory, all bag file in it will be parsed sequentially.

## Check for ros setup srcript
if [ ! -e /opt/ros/melodic/setup.bash ]; then
    echo "Modify path of ROS setup.bash at line 58 in $0, from /opt/ros/melodic/setup.bash to your own."
    exit 0
fi

## Check for virtual environment python 3
if [ ! -e ~/.virtualenvs/dg_venv3.6/bin/activate ]; then
    echo "Modify path of virtualenv of python3 at line 71 in $0, from ~/.virtualenvs/dg_venv3.6/bin/activate to your own."
    exit 0
fi

if [ ! "$1" ]; then
    echo "Usage : $0 rosbag_file_to_be_parsed.bag"
    exit 0
fi

rosbag_file=${1}

if [ ! -e ${rosbag_file} ]; then
    echo "${rosbag_file} is not exists"
    exit 0
fi

## Get this script absolute path
CWD=`pwd`
SCRIPT_PATH=`dirname "$0"`
cd ${SCRIPT_PATH}
SRCPATH=`pwd`
#SRCPATH="/home/test/dg_bin"
echo "$SRCPATH is $0's parents directory"
cd $CWD

poses_latlon_compare_file="poses_latlon_robot_to_compare.txt"
pose_latlon_file="poses_latlon_robot.txt"
pose_utm_file="poses_utm_robot.txt"

function run_parse(){
    local IF=$1
    local OD=$2
#       --pose_only \
#       --unique_jpgname \
#       --gps_topic=/ascen_gps/fix \
#       --gps_topic=/ascen_fix \
#       --gps_topic=/antro2linux_gps \
#       --gps_topic=/novatel_fix \
#       --gps_topic=/gps/fix \
    python2 ${SRCPATH}/parser_bag_py2_7.py --bag_file=$IF \
        --output_dir=$OD --pose_utm_file=${pose_utm_file}\
        --init_skip_meter=0 \
        --sec_per_frame=0.1 \
        --uvc_topic=/uvc_camera/image_raw/compressed \
        --omni_topic=/theta360z1_raw \
        --gps_topic=/antro2linux_gps \
        --imu_topic=/imu/data \
        --enc_left_topic=/dg_encoder/left_tick_data --enc_right_topic=/dg_encoder/right_tick_data \
        --pose_latlon_file=${pose_latlon_file}
}

function run_draw_map(){
    local IF=$1
    local OF=$2
    python3 ${SRCPATH}/draw_point_on_map.py --coord=latlon \
        --ifname=$IF/${pose_latlon_file} \
        --ofname=$OF \
        --dispratio=1.0 \
        --zoom=16
}
		
function run_parse_bag_to_csv(){
    local IF=$1
    local OD=$2
    python2 ${SRCPATH}/parser_bag_py2_7.py --bag_file=$IF \
        --pose_only \
        --csv_only \
        --output_dir=$OD \
        --uvc_topic=/uvc_camera/image_raw/compressed \
        --omni_topic=/theta360z1_raw \
        --gps_topic=/ascen_gps/fix --exact_gps_topic \
        --imu_topic=/imu/data \
        --enc_left_topic=/dg_encoder/left_tick_data --enc_right_topic=/dg_encoder/right_tick_data
}

function run_parse_ascengps(){
    local IF=$1
    local OD=$2
    python2 ${SRCPATH}/parser_bag_py2_7.py --bag_file=$IF \
        --pose_only \
        --output_dir=$OD --pose_utm_file=poses_utm_robot_ascengps.txt \
        --init_skip_meter=0 \
        --sec_per_frame=0.1 \
        --uvc_topic=/uvc_camera/image_raw/compressed \
        --omni_topic=/theta360z1_raw \
        --gps_topic=/ascen_gps/fix --exact_gps_topic \
        --imu_topic=/imu/data \
        --enc_left_topic=/dg_encoder/left_tick_data --enc_right_topic=/dg_encoder/right_tick_data \
        --pose_latlon_file=poses_latlon_robot_ascengps.txt 
}

function run_parse_androidgps(){
    local IF=$1
    local OD=$2
    python2 ${SRCPATH}/parser_bag_py2_7.py --bag_file=$IF \
        --pose_only \
        --output_dir=$OD --pose_utm_file=poses_utm_robot_androidgps.txt \
        --init_skip_meter=0 \
        --sec_per_frame=0.1 \
        --uvc_topic=/uvc_camera/image_raw/compressed \
        --omni_topic=/theta360z1_raw \
        --gps_topic=/andro2linux_gps --exact_gps_topic \
        --imu_topic=/imu/data \
        --enc_left_topic=/dg_encoder/left_tick_data --enc_right_topic=/dg_encoder/right_tick_data \
        --pose_latlon_file=poses_latlon_robot_androidgps.txt
}

function run_parse_ubloxgps(){  # rtk gps
    local IF=$1
    local OD=$2
    python2 ${SRCPATH}/parser_bag_py2_7.py --bag_file=$IF \
        --pose_only \
        --output_dir=$OD --pose_utm_file=poses_utm_robot_ubloxgps.txt \
        --init_skip_meter=0 \
        --sec_per_frame=0.1 \
        --uvc_topic=/uvc_camera/image_raw/compressed \
        --omni_topic=/theta_driver_node/image_raw \
        --gps_topic=/ublox_gps/fix --exact_gps_topic \
        --imu_topic=/imu/data \
        --enc_left_topic=/dg_encoder/left_tick_data --enc_right_topic=/dg_encoder/right_tick_data \
        --pose_latlon_file=poses_latlon_robot_ubloxgps.txt
}

function run_draw_map_compare(){
    local IF=$1
    local OF=$2
    python3 ${SRCPATH}/draw_point_on_map.py --coord=latlon \
        --ifname=$IF/${poses_latlon_compare_file} \
        --ofname=$OF \
        --dispratio=1.0 \
        --zoom=16
}

if [ -n "$VIRTUAL_ENV" ];then
    echo "You need to run $0 in python2.7 after deactivate virtual environment"
    echo "Run following :"
    echo "deactivate; $0"
    exit 0
fi

source /opt/ros/melodic/setup.bash
echo "[bash] Extract image from $rosbag_file"
ofdir=`basename -s ".bag" $rosbag_file`
output_dir="extracted/$ofdir"
