#!/bin/bash

## current dir : top/src/vps
cp init_vps_ramdisk.sh ../../bin/.

cd data_vps
ln -sf ../netvlad

## Make and mount ramdisk as a temporary disk
ramdisk="/mnt/ramdisk"
tmpdir="${ramdisk}/.vps_dataset"
datasetdir="netvlad_etri_datasets"

if [ ! -d $ramdisk ];then
	sudo mkdir -p ${ramdisk}
	sudo mount -t tmpfs -o size=4G tmpfs ${ramdisk}
	echo "You can mount ramdisk automatically by adding following line at the end of the /etc/fstab"
	echo "  none /mnt/ramdisk tmpfs defaults,size=4G 0 0"
fi

if [ ! -d $tmpdir ];then
	mkdir $tmpdir
fi

## Install content of mount directory
rm -rf $tmpdir/* # clear ramdisk

## Link the tmpdir(ramdisk) as a dataset directory.
[ -d $datasetdir ] && rm $datasetdir  # Remove existed link directory
ln -sf $tmpdir $datasetdir
