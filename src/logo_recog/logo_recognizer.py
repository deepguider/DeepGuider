import _init_paths
import os
import cv2
import numpy as np
import time
import pickle
import argparse

# added by jylee to resolve "could not create cudnn handle" issue, 2020.7.10
# ref: https://forums.developer.nvidia.com/t/could-not-create-cudnn-handle-cudnn-status-internal-error/74253/3
import tensorflow as tf
cfg = tf.compat.v1.ConfigProto()
cfg.gpu_options.allow_growth = True
tf.keras.backend.set_session(tf.Session(config=cfg))
# end of added by jyee

from tqdm import tqdm
from pathlib import Path
from keras_yolo3.yolo import YOLO

from logo_recog import detect_logo_only, detect_and_match
from utils import construct_DB, load_features, pad_image, load_extractor_model, similarity_cutoff, extract_features


class LogoRecognizer():
    def __init__(self):
        self.sim_threshold = 0.90
        self.output_txt = 'out.txt'
        self.model_path = './model/keras_yolo3/model_data/logo_yolo_weights.h5'
        self.anchors = './model/keras_yolo3/model_data/yolo_anchors.txt'
        self.yolo_classes_path = './logo_data/preprocessed/openset_classes.txt'
        self.confidence = 0.5
        self.gpu_num = 1
        self.recog_model = 'InceptionV3'
        self.DB_path = './model'
        self.classes_path = './logo_data/preprocessed/trained_brands.pkl'
        self.save_image = False
        self.input_path = './logo_data/demo'
        self.result_path = './logo_data/test'
        if not os.path.exists(self.result_path):
            os.mkdir(self.result_path)
        self.DB_list = './logo_data/preprocessed/DB_list.txt'
        
    def initialize_fast(self):
        self.classes_path = './logo_data/preprocessed/trained_brands.pkl'
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
            #img_input, input_labels = pickle.load(f)
            input_feats, input_labels = pickle.load(f)

        # load pre-trained recognition model
        model, preprocessed, input_shape = load_extractor_model(self.recog_model)
        my_preprocess = lambda x: preprocessed(pad_image(x, input_shape))

        #input_feat = extract_features(img_input, model, my_preprocess)
        sim_cutoff, (bins, cdf_list) = similarity_cutoff(input_feats, features, 0.95)

        print("Done...! It tooks {:.3f} mins\n".format((time.time() - start)/60))
        
        self.model_preproc = (yolo, model, my_preprocess)
        self.params = (input_feats, sim_cutoff, bins, cdf_list, input_labels)        
        return True
    
    def apply(self, image, timestamp):
        
        pred, _, t = detect_and_match(image, self.model_preproc, self.params, 
                                      save_img=self.save_image, 
                                      save_img_path=self.result_path,
                                      is_test=True)
        
        return pred, timestamp

# If you want to test using the code below, change the value of is_test in apply() to False.
#logo_recog = LogoRecognizer()
#logo_recog.initialize()
#logo_recog.apply('./logo_data/demo/ibk.jpg', '2020-04-24')