#!/bin/bash

cd data_vps

## Make and mount ramdisk as a temporary disk
tmpdir="/media/ramdisk_dg"
datasetdir="netvlad_etri_datasets"
[ -d $tmpdir ] && echo "$tmpdir exists" || sudo mkdir $tmpdir
mount |grep $tmpdir || sudo mount -t tmpfs -o size=512M tmpfs $tmpdir
rm -rf $tmpdir/* # clear ramdisk
cp -rf ../Dataset_example/netvlad_etri_datasets/* $tmpdir/.

## Link the tmpdir(ramdisk) as a dataset directory.
[ -d $datasetdir ] && rm $datasetdir  # Remove existed link directory
ln -sf $tmpdir $datasetdir

cd ..
mkdir -p data_vps/$datasetdir/qImg/999_newquery
#mkdir -p data_vps/netvlad_etri_datasets/qImg/999_newquery

cd netvlad
/bin/bash 0setup_test_env.sh

echo "If you met error(s) during downlaoding .tar.gz(s), plz. re-run $0"
