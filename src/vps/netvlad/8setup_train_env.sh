#!/bin/bash
#Subject : Downloads pretrained weight and creates symbolic link of Dataset
#ccsmm@etri.re.kr
#2019.10.07

DATASET_LINK=netvlad_v100_datasets
DATASET_DIR=/home/ccsmm/workdir/DB_Repo/Pittsburgh250k/$DATASET_LINK

#WHICH_WEIGHT="PAPER"
WHICH_WEIGHT="ETRI"

chmod +x *.sh


PRETRAINED_DIR=pretrained_checkpoint



function download_weight(){
    PRETRAINED_FILE=$1
	DOWNLOAD_URL=$2
    
    if [ ! -f "./$PRETRAINED_DIR/$PRETRAINED_FILE" ]; then
    
    	#wget -O ~/gdrivedl 'https://f.mjh.nz/gdrivedl'
    	chmod +x gdrivedl
    
    	#Download pretrained weight of vgg16 from ETRI's trained dataset
    	./gdrivedl $DOWNLOAD_URL
    	mkdir -p $PRETRAINED_DIR 
    	mv $PRETRAINED_FILE $PRETRAINED_DIR
    	cd $PRETRAINED_DIR
    	tar -zxvf $PRETRAINED_FILE
    	cd ..
    	echo "================================================="
    	echo "Prepared $PRETRAINED_FILE"
    else
        echo "$PRETRAINED_FILE exists"
    fi
}


PRETRAINED_FILE=vgg16_netvlad_checkpoint_gpu4.tar.gz
DOWNLOAD_URL=https://drive.google.com/open?id=1dvEPo1g_0zAxcnJYL3rBCaExMOq0pQR2
download_weight $PRETRAINED_FILE $DOWNLOAD_URL

#PRETRAINED_FILE=vgg16_netvlad_checkpoint.zip
#DOWNLOAD_URL=https://drive.google.com/open?id=17luTjZFCX639guSVy00OUtzfTQo4AMF2
PRETRAINED_FILE=vgg16_netvlad_checkpoint_gpu1.tar.gz
DOWNLOAD_URL=https://drive.google.com/open?id=1yHeCIcNudj1taTg1t-cZdv4b65PhqfrG
download_weight $PRETRAINED_FILE $DOWNLOAD_URL



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
