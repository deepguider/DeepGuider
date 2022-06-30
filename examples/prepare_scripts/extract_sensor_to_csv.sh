#!/bin/bash
#by ccsmm@etri.re.kr

source ros_common_script.sh ## $rosbag_file, $output_dir, $ofdir, and $poses_latlon_compare_file are defined in ros_common_script.sh
## rosbag_file can be a file or a directory. If it is directory, all bag file in it will be parsed sequentially.

## Parse rosbag_file to extrace gps and images
run_parse_bag_to_csv "$rosbag_file" "$output_dir"
