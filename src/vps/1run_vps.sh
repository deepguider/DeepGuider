#!/bin/bash

source ~/.virtualenvs/dg_venv3.6/bin/activate
#source ~/.virtualenvs/5env3.5/bin/activate
#Check whether visdom is running before begin

PID1=`pgrep -x "visdom"`
PID2=`ps -ef|grep visdom.server|grep python|wc -l`

#if [ -z "$PID1" ] | [ -z "$PID2" ]
#then
#	echo "You need to run visdom before running :"
#	echo "       python -m visdom.server &"
#	echo "If you didn't install visdom,"
#	echo "You need to install and run it in python3.6 virtual environment"
#	exit 0
#else
#	echo "Checking vidsom... : Ok"
#fi

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
			--dbFeat_fname 'data_vps/prebuilt_dbFeat.mat'\
			--save_dbFeat


elif [ ${nGPU} -eq 1 ]; then
	python vps.py \
			--nGPU 1 --resume 'netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint'\
			--dataset $Dataset --cacheBatchSize 4\
			--dbFeat_fname 'data_vps/prebuilt_dbFeat.mat'\
			--save_dbFeat

else
	echo "Oooops. We've prepared pretrained weights for only 1 or 4 of GPU"

fi

#IP=`hostname -I | sed 's/ //g'`
IP=hostname -I |cut -f1 -d" "
echo "You can navigate to http://localhost:8097 to look over the visual results"
echo "or, you can navigate to http://${IP}:8097 on your local browser"

