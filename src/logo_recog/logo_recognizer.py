import _init_paths
import os
import cv2
import numpy as np
import time
import pickle
import argparse

from tqdm import tqdm
from pathlib import Path
from keras_yolo3.yolo import YOLO

from logo_recog import detect_logo_only, detect_and_match
from utils import construct_DB, load_features, pad_image, load_extractor_model, similarity_cutoff, extract_features


class LogoRecognizer():
    def __init__(self):
        self.sim_threshold = 0.90
        self.output_txt = 'out.txt'
        self.model_path = './model/keras_yolo3/logs/0001/trained_weights_final.h5'
        self.anchors = './model/keras_yolo3/model_data/yolo_anchors.txt'
        self.yolo_classes_path = './logo_data/preprocessed/classes.txt'
        self.confidence = 0.5
        self.gpu_num = 1
        self.recog_model = 'InceptionV3'
        self.DB_path = './model'
        self.classes_path = './logo_data/preprocessed/trained_brands.pkl'
        self.save_image = True
        self.input_path = './logo_data/demo'
        self.result_path = './logo_data/test'
        self.DB_list = './logo_data/preprocessed/DB_list.txt'
        
    def initialize_fast(self):
        self.classes_path = './logo_data/preprocessed/kr_brands.pkl'
        self.initialize()
        return True

    def initialize(self):

        print('Initialization in progress...!\n')        
        
        start = time.time()
        yolo = YOLO(**{"model_path": self.model_path, 
            "anchors_path": self.anchors,
            "classes_path": self.yolo_classes_path,
            "score" : self.confidence,
            "gpu_num" : self.gpu_num,
            "model_image_size" : (416, 416),
            })
        
        # load pre-processed features database
        features, _, _ = load_features(self.recog_model)
        with open(self.classes_path, 'rb') as f:
            img_input, input_labels = pickle.load(f)

        # load pre-trained recognition model
        model, preprocessed, input_shape = load_extractor_model(self.recog_model)
        my_preprocess = lambda x: preprocessed(pad_image(x, input_shape))

        input_feat = extract_features(img_input, model, my_preprocess)
        sim_cutoff, (bins, cdf_list) = similarity_cutoff(input_feat, features, 0.95)

        print("Done...! It tooks {:.3f} mins".format((time.time() - start)/60))
        
        self.model_preproc = (yolo, model, my_preprocess)
        self.params = (input_feat, sim_cutoff, bins, cdf_list, input_labels)
        return True        
    
    def apply(self, image, timestamp):
        
        pred, _, t = detect_and_match(image, self.model_preproc,
                                      self.params, 
                                      save_img=self.save_image, 
                                      save_img_path=self.result_path,
                                      is_test=True)
        
        return pred, timestamp
    
#poi_recog = POIRecognizer()
#poi_recog.apply('./logo_data/demo/ibk.jpg', '2020-04-24')