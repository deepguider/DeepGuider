#!/bin/bash

docker_dir="/home/dg/DB_Repo/Naverlabs/TopoMapAPI/imageserver"
## example string) /dev/sda1 on /media/dg/6232-34351 type exfat (rw,nosuid,nodev,relatime,uid=1000,gid=1000,fmask=0022,dmask=0022,iocharset=utf8,namecase=0,errors=remount-ro,uhelper=udisks2)
MOUNTDIR=`mount |grep /media/dg/6 | cut -d " " -f 3`
echo "sdcard is mounted to ${MOUNTDIR}"
sed "s|SDCARD_MOUNTDIR|${MOUNTDIR}|g" ${docker_dir}/docker-compose.yml.sdcard_template > ${docker_dir}/docker-compose.yml

echo ""
echo ""
echo "SDCARD_MOUNTDIR is replaced with ${MOUNTDIR} in ${docker_dir}/docker-compose.yml"
