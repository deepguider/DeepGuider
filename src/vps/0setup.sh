#!/bin/bash

## current dir : top/src/vps/
cd data_vps
ln -sf ../netvlad

## Make and mount ramdisk as a temporary disk
tmpdir="./.tmp"
if [ ! -f $tmpdir ];then
	mkdir $tmpdir
fi
datasetdir="netvlad_etri_datasets"

## Install content of mount directory
rm -rf $tmpdir/* # clear ramdisk
cp -rf ../Dataset_example/netvlad_etri_datasets/* $tmpdir/.

## Link the tmpdir(ramdisk) as a dataset directory.
[ -d $datasetdir ] && rm $datasetdir  # Remove existed link directory
ln -sf $tmpdir $datasetdir

## current dir : top/src/vps/
cd ..
mkdir -p data_vps/$datasetdir/qImg/999_newquery
#mkdir -p data_vps/netvlad_etri_datasets/qImg/999_newquery

## Make query list(current dir : top/src/vps/)
cd Dataset_example/netvlad_etri_datasets/etri_cart_db
ls -d "$PWD"/* > ../../../data_vps/query_list.txt
cd ../../../

## For unit debugging(current dir : top/src/vps/)
# The following command is executed only on Seungmin's PC.
hostname |grep ccsmm > /dev/null&& cp ~/Naverlabs/query_etri_cart/query_list.txt data_vps/.

## current dir : top/src/vps/
cd netvlad
/bin/bash 0setup_test_env.sh

echo "If you met error(s) during downlaoding .tar.gz(s), plz. re-run $0"
