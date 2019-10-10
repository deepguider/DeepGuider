export TMPDIR="/tmp"
export CHKDIR="./checkpoint"
#export PRETRAINED="pretrained_checkpoint/vgg16_netvlad_checkpoint"
export PRETRAINED="pretrained_checkpointi/vgg16_netvlad_checkpoint_gpu4"

# Dataset is hardcoded at pittsburgh.py as follows : "./netvlad_v100_datasets" directory
# --dataPath : Path for centroid data.
# --runsPath : Path to save running data such as intermediate weights.
# --resume : Path to load checkpoint from, for resuming training or testing. such as pretrained weight
#            which has to have files like follows in its children directory :
#                                   ResumeDIR/checkpoints/
#														 flags.json and 
#														 model_best.pth.tar or
#														 checkpoint.pth.tar


# If you want to use pretrained centeroid, replace --dataPath with as follows:
#				--dataPath=netvlad_v100_datasets --runsPath=${CHKDIR}/runs \


python main.py --mode=cluster --arch=vgg16 --pooling=netvlad --num_clusters=64 \
				--resume=${PRETRAINED} \
				--dataPath=${CHKDIR}/data --runsPath=${CHKDIR}/runs \
				--nGPU 4


python main.py --mode=train --arch=vgg16 --pooling=netvlad --num_clusters=64 \
				--resume=${PRETRAINED} \
				--dataPath=${CHKDIR}/data --runsPath=${CHKDIR}/runs \
				--nGPU 4
