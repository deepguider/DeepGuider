#/bin/bash
#Subject : Downloads pretrained weight and creates symbolic link of Dataset
#ccsmm@etri.re.kr
#2019.10.07

DATASET_LINK=netvlad_v100_datasets
DATASET_DIR=/home/ccsmm/workdir/DB_Repo/Pittsburgh250k/$DATASET_LINK
WEIGHT_FILE=vgg16_netvlad_checkpoint.zip
WEIGHT_DIR=vgg16_netvlad_checkpoint

chmod +x *.sh

if [ ! -f "./$WEIGHT_DIR/$WEIGHT_FILE" ]; then

	#wget -O ~/gdrivedl 'https://f.mjh.nz/gdrivedl'
	chmod +x gdrivedl

	#Download pretrained weight of vgg16 from https://drive.google.com/open?id=17luTjZFCX639guSVy00OUtzfTQo4AMF2
	./gdrivedl https://drive.google.com/open?id=17luTjZFCX639guSVy00OUtzfTQo4AMF2
	mkdir -p $WEIGHT_DIR 
	mv $WEIGHT_FILE $WEIGHT_DIR
	cd $WEIGHT_DIR
	unzip vgg16_netvlad_checkpoint.zip
	cd ..
	echo "================================================="
	echo "Prepared $WEIGHT_FILE"
else
    echo "$WEIGHT_FILE exists"
fi


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
