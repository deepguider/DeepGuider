#!/bin/bash

## Script from https://github.com/deepguider/dg_cart_ros


## Git clone sensor ros file
git clone https://github.com/deepguider/dg_cart_ros.git src/dg_cart_ros

## Build and install dg_cart_ros
source /opt/ros/melodic/setup.bash
catkin_make install

## Add udev rules for accessing and mounting sensor devices
sudo cp src/dg_cart_ros/udev.rules /etc/udev/rules.d/99-dg-device.rules


## Run sensor node with following command, bagfile will be saved at home directory
# roslaunch dg_cart_ros dg_record_sensor.launch
