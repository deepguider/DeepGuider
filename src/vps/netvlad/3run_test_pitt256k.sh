export TMPDIR="/tmp"
export CHKDIR="./checkpoint"

python main.py --mode=test --split=val --resume=vgg16_netvlad_checkpoint \
			   --nGPU 1 --nocuda
