#!/bin/bash

cd data_vps
ln -sf ../Dataset_example/netvlad_etri_datasets
ln -sf ../netvlad
cd ..
mkdir -p data_vps/netvlad_etri_datasets/qImg/999_newquery

cd netvlad
/bin/bash 0setup_test_env.sh


echo "If you meet error during installing this, plz. re-run $0"
