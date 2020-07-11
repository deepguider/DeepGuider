'''For cosine similarity'''

import os
import cv2
import pickle
import argparse

from pathlib import Path
from tqdm import tqdm
from utils import extract_features, load_extractor_model, pad_image


def main(args):
    
    img_input, input_feats, input_labels = [], [], []
    model, preprocessed, input_shape = load_extractor_model(args.recog_model)
    my_preprocess = lambda x: preprocessed(pad_image(x, input_shape))
    
    cat_path = list(Path(args.roi_path).iterdir())
    for cat in tqdm(cat_path):
        label = os.path.basename(str(cat))
        imgs = list(Path(cat).iterdir())
        for img in imgs:
            img = cv2.imread(str(img))
            if img is None:
                continue
            img_input.append(img)
            input_labels.append(label)
    
    input_feats = extract_features(img_input, model, my_preprocess)
    
    with open(os.path.join(args.result_path, 'trained_brands.pkl'), 'wb') as f:
        pickle.dump((input_feats, input_labels), f)
    
    print('Done...!')
    
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--roi_path', type=str, 
            default='./logo_data/kr_brandROIs', # logos-in-the-wild dataset
            help='Path to directory contatining brand ROIs')
    parser.add_argument('--result_path', type=str,
            default='./logo_data/preprocessed',
            help='Path to save the preprocessed data')
    parser.add_argument('--model_path', type=str,
            #default='./model/keras_yolo3/logs/0001/trained_weights_final.h5',
            default='./model/keras_yolo3/model_data/logo_yolo_weights.h5',
            help = 'Path to Yolo model weight file')
    parser.add_argument('--anchors', type=str,
            default='./model/keras_yolo3/model_data/yolo_anchors.txt',
            help = 'Path to Yolo anchors')
    parser.add_argument('--yolo_classes_path', type=str,
            default='./logo_data/preprocessed/openset_classes.txt',
            help = 'Path to Yolo class specifications')
    parser.add_argument('--gpu_num', type=int, default=1,
            help = 'Number of GPU to use')
    parser.add_argument('--confidence', type=float, dest='score', default=0.05,
            help = 'Yolo object confidence threshold above which to show predictions') 
    parser.add_argument('--recog_model', type=str, default='InceptionV3',
            help = 'Select the recognition model', 
            choices = ['InceptionV3', 'VGG16'])
    args = parser.parse_args()
    
    main(args)
