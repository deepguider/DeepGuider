#! /bin/bash
echo " - downloading pretrained YOLOv3 weights..."

# move to ./keras_yolo3, download YOLO weigths
cd ./keras_yolo3/model_data/

echo "Downloading https://logohunters3.s3-us-west-2.amazonaws.com/yolo_weights_logos.h5 in model/keras_yolo3"
wget logohunters3.s3-us-west-2.amazonaws.com/yolo_weights_logos.h5

# move to model, download pre-computed logo features
cd ../../
echo "Downloading https://logohunters3.s3-us-west-2.amazonaws.com/inception_logo_features_200_trunc2.hdf5 in model/"
wget logohunters3.s3-us-west-2.amazonaws.com/inception_logo_features_200_trunc2.hdf5

echo "Downloading https://logohunters3.s3-us-west-2.amazonaws.com/vgg16_logo_features_128.hdf5 in model/"
wget logohunters3.s3-us-west-2.amazonaws.com/vgg16_logo_features_128.hdf5
