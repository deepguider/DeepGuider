import _init_paths
import argparse
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import pickle
import utils
import time

from PIL import Image
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGE = True

from timeit import default_timer as timer
from keras_yolo3.yolo import YOLO
from logos import detect_logo, match_logo, detect_logo_demo, detect_and_match
from similarity import load_brands_compute_cutoffs
from utils import load_extractor_model, load_features, model_flavor_from_name, parse_input
from pathlib import Path

class POIRecognizer:
    def __init__(self):
        self.sim_threshold = 0.90
        self.output_txt = 'out.txt'

    def initialize(self):
        self.sim_threshold = 0.90
        self.output_txt = 'out.txt'
        
        filename = './model/inception_logo_features_200_trunc2.hdf5'
        print('Initialization in progress...!\n')        
        start = time.time()
        yolo = YOLO(**{"model_path": './model/keras_yolo3/model_data/yolo_weights_logos.h5',
            "anchors_path": './model/keras_yolo3/model_data/yolo_anchors.txt',
            "classes_path": './data/preprocessed/classes.txt',
            "score" : 0.05,
            "gpu_num" : 1,
            "model_image_size" : (416, 416),
            })
        # get Inception/VGG16 model and flavor from filename
        model_name, flavor = model_flavor_from_name(filename)
        
        ## load pre-processed features database
        features, brand_map, input_shape = load_features(filename)

        ## load inception model
        model, preprocess_input, input_shape = load_extractor_model(model_name, flavor)
        my_preprocess = lambda x: preprocess_input(utils.pad_image(x, input_shape))

        with open('./data/preprocessed/trained_brands.pkl', 'rb') as f:
            img_input, input_labels = pickle.load(f)

        (img_input, feat_input, sim_cutoff, (bins, cdf_list)) = load_brands_compute_cutoffs(img_input, (model, my_preprocess), features, self.sim_threshold)
        self.model_preproc = (yolo, model, my_preprocess)
        self.input_preproc = (feat_input, sim_cutoff, bins, cdf_list, input_labels)
        
        print('Done! It tooks {:.2f} mins.\n'.format((time.time() - start)/60))
        return True

    def apply(self, image, timestamp):
        pred, timestamp = detect_and_match(self.model_preproc, self.input_preproc, image, timestamp, save_img=True)
        return pred, timestamp
        
