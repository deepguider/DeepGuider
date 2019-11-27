#!/bin/bash
DATASET_LINK1=netvlad_v100_datasets
DATASET_DIR1=/home/ccsmm/workdir/DB_Repo/Pittsburgh250k/$DATASET_LINK1
DATASET_LINK2=netvlad_etri_datasets
DATASET_DIR2=/home/ccsmm/workdir/DB_Repo/Pittsburgh250k/$DATASET_LINK2
PRETRAINED_DIR=pretrained_checkpoint

function func_slink(){
	LINK=$1
	DIR=$2
	if [ ! -d "$DIR" ]; then
		echo "ERROR : You need to modify location of Dataset in this script file"
		echo "      : Refer Dataset_example directory"
		echo "$DIR"
	else
		if [ ! -L "$LINK" ]; then
			ln -s $DIR $LINK
		fi

		echo "================================================="
		echo "Created symbolic link of $DATASET_LINK"
	fi
}


func_slink $DATASET_LINK1 $DATASET_DIR1
func_slink $DATASET_LINK2 $DATASET_DIR2


cd netvlad
/bin/bash 0setup_env.sh
