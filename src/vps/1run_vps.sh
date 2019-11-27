#!/bin/bash

Dataset="deepguider"
#Dataset="'pittsburgh'"
nGPU=`python -c "import torch;print(torch.cuda.device_count())"`
#nGPU=1

if [ ${nGPU} -eq 4 ]; then
	python vps.py \
			--nGPU 4 --resume 'netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint_gpu4'\
			--dataset $Dataset

elif [ ${nGPU} -eq 1 ]; then
	python vps.py \
			--nGPU 1 --resume 'netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint'\
			--dataset $Dataset

else
	echo "Oooops. We've prepared pretrained weights for only 1 or 4 of GPU"

fi
