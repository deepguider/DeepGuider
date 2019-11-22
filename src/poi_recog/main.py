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

sim_threshold = 0.90
output_txt = 'out.txt'


def test(filename, timestamp):
    """
    Test function: runs pipeline for a small set of input images and input
    brands.
    """
    yolo = YOLO(**{"model_path": './model/keras_yolo3/model_data/yolo_weights_logos.h5',
                "anchors_path": './model/keras_yolo3/model_data/yolo_anchors.txt',
                "classes_path": './data/preprocessed/classes.txt',
                "score" : 0.05,
                "gpu_num" : 1,
                "model_image_size" : (416, 416),
                })
    save_img_logo, save_img_match = True, True
    
    test_dir = os.path.join(os.path.dirname(__file__), 'data/test')

    # get Inception/VGG16 model and flavor from filename
    model_name, flavor = model_flavor_from_name(filename)
    ## load pre-processed features database
    features, brand_map, input_shape = load_features(filename)

    ## load inception model
    model, preprocess_input, input_shape = load_extractor_model(model_name, flavor)
    my_preprocess = lambda x: preprocess_input(utils.pad_image(x, input_shape))
     
    images = [ p for p in os.listdir(os.path.join(test_dir, 'input/'))] #if p.endswith('.jpg')]
    images_path = [ os.path.join(test_dir, 'input/', p) for p in images]

    #with open('./data/preprocessed/input_paths.pkl', 'r') as f:
    #    input_paths = pickle.load(f)
    #(img_input, feat_input, sim_cutoff, (bins, cdf_list)) = load_brands_compute_cutoffs(
    #                            input_paths, (model, my_preprocess), features, sim_threshold)

    start = timer()
    img_size_list = []
    candidate_len_list = []
    for i, img_path in enumerate(images_path):
        outtxt = img_path

        ## find candidate logos in image
        prediction, image = detect_logo(yolo, img_path, save_img = True,
                                          save_img_path = test_dir, postfix='_logo')

        ## match candidate logos to input
        #logo_txt = match_logo(image, prediction, (model, my_preprocess),
        #         outtxt, (feat_input, sim_cutoff, bins, cdf_list, input_labels),
        #         save_img = True, save_img_path=test_dir, timing=True)

        img_size_list.append(np.sqrt(np.prod(image.size)))
        candidate_len_list.append(len(prediction))
        #times_list.append(times)

    end = timer()
    print('Processed {} images in {:.1f}sec - {:.1f}FPS'.format(
            len(images_path), end-start, len(images_path)/(end-start)
           )) 
    print(f'Timestamp : {timestamp}')


def initialize(filename):
   
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

    (img_input, feat_input, sim_cutoff, (bins, cdf_list)) = load_brands_compute_cutoffs(                                    
                                            img_input, (model, my_preprocess), features, sim_threshold)
    print('Done! It tooks {:.2f} mins.\n'.format((time.time() - start)/60))

    return (yolo, model, my_preprocess), (feat_input, sim_cutoff, bins, cdf_list, input_labels)


if __name__ == '__main__':
    timestamp = 123.456
    filename = './model/inception_logo_features_200_trunc2.hdf5'
    #test(filename, timestamp)
    model_preproc, input_preproc = initialize(filename)
    test_path = list(Path('./data/test/input/').iterdir())
    #print(len(test_path))
    start = time.time()
    for path in test_path:
        img = cv2.imread(str(path))
        pred, timestamp = detect_and_match(model_preproc, input_preproc, 
                                       img, str(path), timestamp, save_img=True)
        print(pred)
        #print(timestamp)
    print('Logo detection and recognition complete! It tooks {:.2f} FPS'.format(len(test_path)/(time.time() - start)))

    '''
    image = cv2.imread('./data/test/input/test_starbucks.png')
    pred, timestamp = detect_logo_demo('./data/test/input/test_starbucks.png', timestamp)
    print(pred, timestamp)
    '''
