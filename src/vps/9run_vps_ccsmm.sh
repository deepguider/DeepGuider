#!/bin/bash

## Run your virtual environment activation script
source ~/.virtualenvs/3venv3.5/bin/activate;

Dataset="deepguider"
#Dataset="'pittsburgh'"
#nGPU=`python -c "import torch;print(torch.cuda.device_count())"`
nGPU=1
#nGPU=4


#			--save_dbFeat
#			--use_saved_dbFeat
#			--save_qFeat
#			--use_saved_qFeat

if [ ${nGPU} -eq 4 ]; then
	python vps.py \
			--nGPU 4 --resume 'netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint_gpu4'\
			--dataset $Dataset --cacheBatchSize 36\
			--dbFeat_fname 'netvlad_etri_datasets/prebuilt_dbFeat.mat'\
			--use_saved_dbFeat
#			--save_dbFeat


elif [ ${nGPU} -eq 1 ]; then
	python vps.py \
			--nGPU 1 --resume 'netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint'\
			--dataset $Dataset --cacheBatchSize 4\
			--dbFeat_fname 'netvlad_etri_datasets/prebuilt_dbFeat.mat'\
			--use_saved_dbFeat
#			--save_dbFeat
#			--use_saved_dbFeat

else
	echo "Oooops. We've prepared pretrained weights for only 1 or 4 of GPU"

fi
