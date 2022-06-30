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
	## Indoor streetview for VPS in prebuilt weight mode
	echo "Flushing and copying indoor streetview images to ${TARGET}/data_vps/netvlad_etri_datasets/dbImg/StreetView/."
	rm -rf ${TARGET}/data_vps/netvlad_etri_datasets/dbImg/StreetView/*
	mkdir -p ${TARGET}/data_vps/netvlad_etri_datasets/dbImg/StreetView
    ln -sf ${SRCPATH}/data_vps/netvlad_etri_datasets_indoor_etri12b_1way/dbImg/StreetView/* ${TARGET}/data_vps/netvlad_etri_datasets/dbImg/StreetView/.
    ln -sf ${SRCPATH}/data_vps/prebuilt_dbFeat_1way.mat ${TARGET}/data_vps/prebuilt_dbFeat.mat

	echo "If you got error, re-run this after running catkin_ws/init_vps_ramdisk.sh"
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

