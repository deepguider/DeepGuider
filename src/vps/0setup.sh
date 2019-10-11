#!/bin/bash
DATASET_LINK=netvlad_v100_datasets
DATASET_DIR=/home/ccsmm/workdir/DB_Repo/Pittsburgh250k/$DATASET_LINK
PRETRAINED_DIR=pretrained_checkpoint

if [ ! -d "$DATASET_DIR" ]; then
	echo "ERROR : You need to modify location of Dataset in this script file"
	echo "$DATASET_DIR"
else
	if [ ! -L "$DATASET_LINK" ]; then
		ln -s $DATASET_DIR
	fi
	echo "================================================="
	echo "Created symbolic link of $DATASET_LINK"
fi

cd netvlad
/bin/bash 0setup_env.sh
