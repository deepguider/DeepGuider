#!/bin/bash
#by ccsmm@etri.re.kr

source ros_common_script.sh ## $rosbag_file, $output_dir, $ofdir, and $poses_latlon_compare_file are defined in ros_common_script.sh

## rosbag_file can be a file or a directory. If it is directory, all bag file in it will be parsed sequentially.

## Parse rosbag_file to extrace gps and images

run_parse_androidgps "$rosbag_file" "$output_dir"
run_parse_ascengps "$rosbag_file" "$output_dir"

## Save gps to ${poses_latlon_compare_file}
extracted_dir="$output_dir/uvc_image"
if [ ! -x ${extracted_dir} ];then
	extracted_dir="$output_dir/omni_image"
fi
echo " >>> Save raw gps into ${extracted_dir}/${poses_latlon_compare_file}"

echo " >>> Blue color points are for 1st detected [GPS] topic"
echo "color blue" > ${extracted_dir}/${poses_latlon_compare_file}
if [ -e ${extracted_dir}/poses_latlon_robot_androidgps.txt ];then
	cat ${extracted_dir}/poses_latlon_robot_androidgps.txt >> ${extracted_dir}/${poses_latlon_compare_file}
fi

echo " >>> Red color points are for 2nd detected [GPS] topic"
if [ -e ${extracted_dir}/poses_latlon_robot_ascengps.txt ];then
	echo "color red" >> ${extracted_dir}/${poses_latlon_compare_file}
	cat ${extracted_dir}/poses_latlon_robot_ascengps.txt >> ${extracted_dir}/${poses_latlon_compare_file}
fi

source ~/.virtualenvs/dg_venv3.6/bin/activate
echo "Draw map from $rosbag_file"

## Draw map
map_name="map/${ofdir}_compareImgSyncedGPS"

run_draw_map_compare "$extracted_dir" "$map_name"
