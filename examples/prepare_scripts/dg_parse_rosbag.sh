#!/bin/bash
#by ccsmm@etri.re.kr

# rosbag_file can be a file or a directory. If it is directory, all bag file in it will be parsed sequentially.

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

function run_parse(){
	local IF=$1
	local OD=$2
#		--pose_only \
#		--unique_jpgname \
#		--gps_topic=/ascen_gps/fix
#		--gps_topic=/ascen_fix
#		--gps_topic=/antro2linux_gps
#		--gps_topic=/novatel_fix
#		--gps_topic=/gps/fix
	python ${SRCPATH}/parser_bag_py2_7.py --bag_file=$IF \
		--output_dir=$OD --pose_utm_file=poses_utm_robot.txt \
		--init_skip_meter=0 \
		--sec_per_frame=0.1 \
		--uvc_topic=/uvc_camera/image_raw/compressed \
		--omni_topic=/theta360z1_raw \
		--gps_topic=/antro2linux_gps \
		--imu_topic=/imu/data \
		--pose_latlon_file=poses_latlon_robot.txt 
}

function run_draw_map(){
	local IF=$1
	local OF=$2
	python ${SRCPATH}/draw_point_on_map.py --coord=latlon \
		--ifname=$IF/poses_latlon_robot.txt \
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

## Parse rosbag_file to extrace gps and images
run_parse "$rosbag_file" "$output_dir"

source ~/.virtualenvs/dg_venv3.6/bin/activate
echo "Draw map from $rosbag_file"

extracted_dir="$output_dir/uvc_image"
map_name="map/$ofdir"

## Draw map
run_draw_map "$extracted_dir" "$map_name"

videopath="${extracted_dir}/${ofdir}.avi"
#	--no_display
#    --watermark=ETRI_Building12_Floor7
python ${SRCPATH}/imgs2video.py \
	--no_display \
    --in_imgpath=${extracted_dir} \
    --ext=jpg \
    --out_videopath=${videopath} \
    --fps=15

extracted_dir="$output_dir/omni_image"
videopath="${extracted_dir}/${ofdir}.avi"
python ${SRCPATH}/imgs2video.py \
	--no_display \
    --in_imgpath=${extracted_dir} \
    --ext=jpg \
    --out_videopath=${videopath} \
    --fps=15
