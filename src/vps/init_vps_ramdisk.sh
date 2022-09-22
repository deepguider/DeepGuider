#!/bin/bash
## current dir : ./bin

## Make and mount ramdisk as a temporary disk
ramdisk="/mnt/ramdisk"
already_exist=`mount |grep $ramdisk`

#if [ ! -d $ramdisk ];then
if [ ! "$already_exist" ];then
    sudo mkdir -p ${ramdisk}
    sudo mount -t tmpfs -o size=4G tmpfs ${ramdisk}
    sudo chmod 777 ${ramdisk} -R
    mkdir -p /mnt/ramdisk/.vps_dataset/dbImg/StreetView
    echo "You can mount ramdisk automatically by adding the following line at the end of the /etc/fstab"
    echo "  none /mnt/ramdisk tmpfs defaults,size=4G 0 0"

else
    echo "This allows all users to have access to ${ramdisk}"
    sudo chmod 777 ${ramdisk} -R
fi

ln -sf /mnt/ramdisk/.vps_dataset data_vps/netvlad_etri_datasets
