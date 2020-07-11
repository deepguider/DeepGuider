## Logo Detection and Recognition

### Getting started
We uses the Logos-in-the-wild dataset with voc format as training dataset which can be requested via email directly from the authors of the paper, [arXiv:1710.10891](https://arxiv.org/abs/1710.10891). This dataset includes 11,054 images with 32,850 bounding boxes for a total of 871 brands.
Download the dataset and put it in the `./logo_data` folder.

And now we extract this dataset from the urls, refine it, and format it for training and testing.
```
python litw_constructor.py
cd logo_data/LogosInTheWild-v2/scripts
python create_clean_dataset.py --in ../data --out ../cleaned_data --roi
cd ../../..
python voc_format.py
```

And load pre-trained YOLO model weights.
```
cd model/keras_yolo3
wget https://pjreddie.com/media/files/yolov3.weights
wget https://pjreddie.com/media/files/yolov3-tiny.weights
python convert.py yolov3.cfg yolov3.weights model_data/yolo_weights.h5
python convert.py yolov3-tiny.cfg yolov3-tiny.weights model_data/yolo_tiny_weights.h5
cd ../..
```

If you want to use a pre-trained model, download the pre-trained model from [google drive](https://drive.google.com/drive/folders/1LqsGfV-KBnP1tjH94LselZer_K4tx6in?usp=sharing) and place the downloaded file in `./model`.

```
cd model
tar -xvzf model.tar.gz
mv trained_brands.pkl ../logo_data/preprocessed
mv logo_yolo_weights.h5 keras_yolo3/model_data
cd ..
```

### For training
```
python train.py
```

### For testing
Put input images into `./logo_data/input`.
```
python test.py --mode detect # for only detecting
python test.py --mode recog  # for detecting and recognizing
python test.py --mode DB     # for constructing the database of features
```