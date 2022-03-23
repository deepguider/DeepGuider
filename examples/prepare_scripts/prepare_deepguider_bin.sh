#!/bin/bash
CWD=`pwd`

## Get script src's path
SCRIPT_PATH=`dirname "$0"`
cd ${SCRIPT_PATH}
SRCPATH=`pwd`
#SRCPATH="/home/test/dg_bin"
echo "$SRCPATH is $0's parents directory"

cd ${CWD}

if [ ! "${1}" ]; then
    TARGET="./DeepGuider/bin"
else
    TARGET="${1}/bin"
fi

if [ -e ${TARGET} ]; then
	## Initialize example's setup.
	cd ${TARGET};cd ../examples;./clean_setup_all.sh;./setup_all.sh;cd ${CWD}

	## dg_simple_ros
	ln -sf ${SRCPATH}/recordings ${TARGET}/.  # rosbag data file

    ## Test dataset
    ln -sf ${SRCPATH}/data/ETRI ${TARGET}/data/video
    ln -sf ${SRCPATH}/data/ETRI ${TARGET}/data/.
    ln -sf ${SRCPATH}/data/COEX ${TARGET}/data/.

    ## Video File
    ln -sf ${SRCPATH}/data/191115_ETRI.avi ${TARGET}/data/.

    ## Font
    ln -sf ${SRCPATH}/font ${TARGET}/.
    
	## RoadLR weight
    ln -sf ${SRCPATH}/data_roadlr/ckpt.pth.best.acc96p ${TARGET}/data_roadlr/.
    
	## VPS weight
    ln -sf ${SRCPATH}/data_vps/netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint ${TARGET}/data_vps/netvlad/pretrained_checkpoint/.
    ln -sf ${SRCPATH}/data_vps/netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint_gpu4 ${TARGET}/data_vps/netvlad/pretrained_checkpoint/.
    cp -rf ${SRCPATH}/data_vps/prebuilt_dbFeat_1way.mat ${TARGET}/data_vps/prebuilt_dbFeat.mat

	## VPS : Indoor streetview for VPS in prebuilt weight mode
	echo "Flushing and copying indoor streetview images to ${TARGET}/data_vps/netvlad_etri_datasets/dbImg/StreetView/."
	rm -rf ${TARGET}/data_vps/netvlad_etri_datasets/dbImg/StreetView/*
    #cp -rf ${SRCPATH}/data_vps/netvlad_etri_datasets_indoor_etri12b_1way/dbImg/StreetView/* ${TARGET}/data_vps/netvlad_etri_datasets/dbImg/StreetView/.

	## VPS : Custom dataset instead of Naver image server
	ln -sf ${SRCPATH}/data_vps/custom_dataset ${TARGET}/data_vps/.

    ## Intersection weight
    # ln -sf ${SRCPATH}/data_intersection_cls ${TARGET}/data_intersection_cls  ## This was done in examples/setup_intersection.sh
    ln -sf ${SRCPATH}/data_intersection_cls/v1.10_binary_resnet18_3_best.pth ${TARGET}/data_intersection_cls/.
    ln -sf ${SRCPATH}/data_intersection_cls/weight.pth ${TARGET}/data_intersection_cls/.

	## Localizer

	## OCR
	ln -sf ${SRCPATH}/data_ocr/best_craft.pth ${TARGET}/data_ocr/.
	ln -sf ${SRCPATH}/data_ocr/best_accuracy.pth ${TARGET}/data_ocr/.

	## POI

	## Road Reconizer

	## Guidance

	## Exploration
	ln -sf ${SRCPATH}/data_exp ${TARGET}/.
	rm -rf ${TARGET}/../src/exploration/data_exp
	ln -sf ${SRCPATH}/data_exp ${TARGET}/../src/exploration/data_exp

	## Misc.

	## Check error on return value of above commands
	if [ $? -eq 1 ]; then # Error
		echo "Some error have occured. Did you run ./setup_all.sh in examples directory before running ${0}."
	fi
else
	echo ">>>No target directory exists : ${TARGET}"
	echo ">>>Run at parent directory of DeepGuider or specify your own directory."
	echo "Usage : "
	echo "        $0 ./myDeepGuider"
fi

