export TMPDIR="/tmp"
export CHKDIR="./checkpoint"
#export PRETRAINED="pretrained_checkpoint/vgg16_netvlad_checkpoint"
export PRETRAINED="pretrained_checkpoint/vgg16_netvlad_checkpoint_gpu4"

# Dataset is hardcoded at pittsburgh.py as follows : "./netvlad_v100_datasets" directory
# --dataPath : Path for centroid data.
# --runsPath : Path to save running data such as intermediate weights.
# --resume : Path to load checkpoint from, for resuming training or testing. such as pretrained weight
#            which has to have files like follows in its children directory :
#                                   ResumeDIR/checkpoints/
#														 flags.json and 
#														 model_best.pth.tar or
#														 checkpoint.pth.tar


python main.py --mode=test --split=val --resume=${PRETRAINED} \
			--dataPath=${CHKDIR}/data --runsPath=${CHKDIR}/runs \
			--nGPU 4 --cacheBatchSize 4
#			   --nocuda
