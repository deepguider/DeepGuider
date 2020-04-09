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


def main(args):

    yolo = YOLO(**{'model_path': args.model_path,
                   'anchors_path': args.anchors,
                   'classes_path': args.yolo_classes_path,
                   'score': args.score,
                   'gpu_num': args.gpu_num,
                   'model_image_size': (416, 416),
                   })

    if args.mode.lower() == 'detect':
        cat_list = list(Path(args.input_path).iterdir())

        elapsed_t = 0
        img_cnt = 0
        for cat in cat_list:
            img_list = list(Path(cat).iterdir())
            for img_path in img_list:
                pred, _, t = detect_logo_only(str(img_path), yolo,
                                 save_img=args.save_img, save_img_path=args.result_path)
                for p in pred:
                    x1 = p[0]
                    y1 = p[1]
                    x2 = p[2]
                    y2 = p[3]
                    roi = im[y1:y2, x1:x2]
                if t == 0:
                    continue
                elapsed_t += t
                img_cnt += 1
        print('\nDetecting Logos from images completed! It tooks {:.3f} FPS'.format(img_cnt/elapsed_t))

    elif args.mode.lower() == 'recog':
        model_preproc, params = initialize(yolo, args.recog_model, args.DB_path)
        img_list = list(Path(args.input_path).iterdir())

        elapsed_t = 0
        img_cnt = 0
        for img_path in img_list:
            _, _, t = detect_and_match(str(img_path), model_preproc, params,
                             save_img=args.save_img, save_img_path=args.result_path)
            if t == 0:
                continue
            elapsed_t += t
            img_cnt += 1
        print('\nDetecting and Recognizing Logos from images completed! It tooks {:.3f} FPS'.format(img_cnt/elapsed_t))

    else: # args.model == 'DB'
        construct_DB(args.DB_list, args.recog_model, args.DB_path)

    print("Done...!")


def initialize(yolo, model_name, DB_path):
    print("\n\nInitialization in progress...!\n")
    start = time.time()

    # load pre-processed features database
    features, _, _ = load_features(model_name)
    with open(args.classes_path, 'rb') as f:
        img_input, input_labels = pickle.load(f)

    # load pre-trained recognition model
    model, preprocessed, input_shape = load_extractor_model(model_name)
    my_preprocess = lambda x: preprocessed(pad_image(x, input_shape))

    input_feat = extract_features(img_input, model, my_preprocess)
    sim_cutoff, (bins, cdf_list) = similarity_cutoff(input_feat, features, 0.95)

    print("Done...! It tooks {:.3f} mins".format((time.time() - start)/60))

    return (yolo, model, my_preprocess), (input_feat, sim_cutoff, bins, cdf_list, input_labels)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='recog',
            help = 'Logo only detection? or proceed to recognition? or construct DB?',
            choices = ['recog', 'detect', 'DB'])
    parser.add_argument('--input_path', type=str, default='./data/input',
            help = 'Path to load test images')
    parser.add_argument('--result_path', type=str, default='./data/test',
            help = 'Path to save the annotated images')
    parser.add_argument('--DB_path', type=str, default='./model',
            help = 'Path to load or save the DB of features')
    parser.add_argument('--DB_list', type=str,
            default='./data/preprocessed/DB_list.txt',
            help = 'Path to load the list of training dataset with labels.')
    parser.add_argument('--model_path', type=str,
            default='./model/keras_yolo3/logs/0001/trained_weights_final.h5',
            #default='./model/keras_yolo3/model_data/yolo_weights_logos.h5',
            help = 'Path to Yolo model weight file')
    parser.add_argument('--anchors', type=str,
            default='./model/keras_yolo3/model_data/yolo_anchors.txt',
            help = 'Path to Yolo anchors')
    parser.add_argument('--yolo_classes_path', type=str,
            default='./data/preprocessed/openset_classes.txt',
            help = 'Path to Yolo class specifications')
    parser.add_argument('--classes_path', type=str,
            default='./data/preprocessed/trained_brands.pkl',
            help = 'Path to load the brandROIs with labels')
    parser.add_argument('--recog_model', type=str, default='InceptionV3',
            help = 'Select the recognition model', choices = ['InceptionV3', 'VGG16'])
    parser.add_argument('--save_img', type=bool, default=True,
            help = 'Do you want to save the annotated image?')
    parser.add_argument('--gpu_num', type=int, default=1,
            help = 'Number of GPU to use')
    parser.add_argument('--confidence', type=float, dest='score', default=0.05,
            help = 'Yolo object confidence threshold above which to show predictions')
    args = parser.parse_args()

    main(args)
