export TMPDIR="/tmp"
export CHKDIR="./checkpoint"

python main.py --mode=cluster --arch=vgg16 --pooling=netvlad --num_clusters=64 \
				--dataPath=netvlad_v100_datasets --runsPath=${CHKDIR}/runs


python main.py --mode=train --arch=vgg16 --pooling=netvlad --num_clusters=64 \
				--dataPath=${CHKDIR}/data --runsPath=${CHKDIR}/runs
