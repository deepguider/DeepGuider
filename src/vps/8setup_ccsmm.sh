#!/bin/bash


DATASET_DIR1=~/workdir/DB_Repo/Pittsburgh250k/netvlad_v100_datasets
DATASET_DIR2=~/workdir/DB_Repo/NaverLabs/streetview/netvlad_etri_datasets
DATASET_DIR3=~/workdir/DB_Repo/ETRIDB_Cart/netvlad_etri_datasets

cd data_vps
ln -sf ${DATASET_DIR1}
ln -sf ${DATASET_DIR2}
ln -sf ../netvlad
cd ..
mkdir -p data_vps/netvlad_etri_datasets/qImg/999_newquery

cd netvlad
/bin/bash 8setup_train_env.sh

echo "If you meet error during installing this, plz. re-run $0"
