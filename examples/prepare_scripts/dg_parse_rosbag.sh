#!/bin/bash
#by ccsmm@etri.re.kr

source ros_common_script.sh ## $rosbag_file, $output_dir, $ofdir, and $poses_latlon_compare_file are defined in ros_common_script.sh
## rosbag_file can be a file or a directory. If it is directory, all bag file in it will be parsed sequentially.

## Parse rosbag_file to extrace gps and images
run_parse "$rosbag_file" "$output_dir"

## Draw map
source ~/.virtualenvs/dg_venv3.6/bin/activate
echo "Draw map from $rosbag_file"
extracted_dir="$output_dir/uvc_image"
if [ ! -x ${extracted_dir}/${pose_latlon_file} ];then
    extracted_dir="$output_dir/omni_image"
fi
map_name="map/${ofdir}_ImgSyncedGPS"
run_draw_map "$extracted_dir" "$map_name"

## Convert uvc_images to video files
extracted_dir="$output_dir/uvc_image"
if [ -x ${extracted_dir} ];then
	videopath="${extracted_dir}/${ofdir}.avi"
	#	--no_display
	#    --watermark=ETRI_Building12_Floor7
	python ${SRCPATH}/imgs2video.py \
		--no_display \
	    --in_imgpath=${extracted_dir} \
	    --ext=jpg \
	    --out_videopath=${videopath} \
	    --fps=15
fi

## Convert omni-directional images to video file
extracted_dir="$output_dir/omni_image"
if [ -x ${extracted_dir} ];then
	videopath="${extracted_dir}/${ofdir}.avi"
	python ${SRCPATH}/imgs2video.py \
		--no_display \
	    --in_imgpath=${extracted_dir} \
	    --ext=jpg \
	    --out_videopath=${videopath} \
	    --fps=15
fi
