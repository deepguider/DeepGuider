#!/bin/bash
DATASET_LINK1=netvlad_etri_datasets
DATASET_DIR1=./Dataset_example/$DATASET_LINK1
PRETRAINED_DIR=pretrained_checkpoint

function func_slink(){
	LINK=$1
	DIR=$2
	if [ ! -d "$DIR" ]; then
		echo "ERROR : You need to modify location of Dataset in this script file"
		echo "$DIR"
	else
		if [ ! -L "$LINK" ]; then
			ln -s $DIR $LINK
		fi

		mkdir -p $LINK/qImg/999_newquery



		echo "================================================="
		echo "Created symbolic link of $DATASET_LINK"
	fi
}


func_slink $DATASET_LINK1 $DATASET_DIR1


cd netvlad
/bin/bash 0setup_env.sh
