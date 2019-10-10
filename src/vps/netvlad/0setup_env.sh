#/bin/bash
#Subject : Downloads pretrained weight and creates symbolic link of Dataset
#ccsmm@etri.re.kr
#2019.10.07

DATASET_LINK=netvlad_v100_datasets
DATASET_DIR=/home/ccsmm/workdir/DB_Repo/Pittsburgh250k/$DATASET_LINK
PRETRAINED_DIR=pretrained_checkpoint

#WHICH_WEIGHT="PAPER"
WHICH_WEIGHT="ETRI"

chmod +x *.sh

if [ $WHICH_WEIGHT == "PAPER" ]; then #Pretrained weight of paper's author released
    PRETRAINED_FILE=vgg16_netvlad_checkpoint.zip
    
    if [ ! -f "./$PRETRAINED_DIR/$PRETRAINED_FILE" ]; then
    
    	#wget -O ~/gdrivedl 'https://f.mjh.nz/gdrivedl'
    	chmod +x gdrivedl
    
    	#Download pretrained weight of vgg16 from https://drive.google.com/open?id=17luTjZFCX639guSVy00OUtzfTQo4AMF2
    	./gdrivedl https://drive.google.com/open?id=17luTjZFCX639guSVy00OUtzfTQo4AMF2
    	#Download pretrained weight of vgg16 from ETRI's trained dataset
#    	./gdrivedl https://drive.google.com/open?id=1OlhU68w1ZZMUZozMLqsvgVnjhzpe02-a
    	mkdir -p $PRETRAINED_DIR 
    	mv $PRETRAINED_FILE $PRETRAINED_DIR
    	cd $PRETRAINED_DIR
    	unzip $PRETRAINED_FILE
    	cd ..
    	echo "================================================="
    	echo "Prepared $PRETRAINED_FILE"
    else
        echo "$PRETRAINED_FILE exists"
    fi
else #Pretrained weight of ETRI released using streetviews near ETRI & KAIST
    PRETRAINED_FILE=vgg16_netvlad_checkpoint_gpu4.tar.gz
    
    if [ ! -f "./$PRETRAINED_DIR/$PRETRAINED_FILE" ]; then
    
    	#wget -O ~/gdrivedl 'https://f.mjh.nz/gdrivedl'
    	chmod +x gdrivedl
    
    	#Download pretrained weight of vgg16 from ETRI's trained dataset
    	./gdrivedl https://drive.google.com/open?id=1dvEPo1g_0zAxcnJYL3rBCaExMOq0pQR2
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
