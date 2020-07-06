#!/bin/sh
gnome-terminal -e roscore
sleep 2s
#rosbag play ./recordings/2019-11-15-11-40-06.bag
rosbag play -s 500 -r 10 ./recordings/2020-02-19-15-01-53.bag
