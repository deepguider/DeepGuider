import os
import time
import string
import argparse

import torch
import torch.backends.cudnn as cudnn
import torch.utils.data

import numpy as np

from utils import AttnLabelConverter, Averager
from model import Model

from demo import detect_ocr

class OCRRecognizer:
    def __init__(self):

        self.model = None
        self.converter = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.parser = self.setup_parser()
        self.opt = self.parser.parse_args()
        self.args = vars(self.opt)

        # print("~~~~~~~~ Hyperparameters used: ~~~~~~~")
        # for x, y in self.args.items():
        #     print("{} : {}".format(x, y))

        self.__dict__.update(self.args)


    def initialize(self):

        start = time.time()

        # self.image_path = './demo_image/demo_5.png'
        self.saved_model = '/home/sungeun.kim/ocr_recog/saved_models/ocr_train/best_accuracy.pth'
        self.logfilepath = './log_ocr_result.txt'


        """ vocab / character number configuration """
        if self.sensitive:
            self.character = string.printable[:-6]  # same with ASTER setting (use 94 char).

        cudnn.benchmark = True
        cudnn.deterministic = True
        self.num_gpu = torch.cuda.device_count()

        """ model configuration """
        self.converter = AttnLabelConverter(self.character)
        self.num_class = len(self.converter.character)

        if self.rgb:
            self.input_channel = 3

        self.model = Model(self)
        # print('model input parameters', self.imgH, self.imgW, self.num_fiducial, self.input_channel, self.output_channel,
        #       self.hidden_size, self.num_class, self.batch_max_length)

        self.model = torch.nn.DataParallel(self.model).to(self.device)

        # load model
        print('loading pretrained model from %s' % self.saved_model)
        self.model.load_state_dict(torch.load(self.saved_model, map_location=self.device))



        print('Done! It tooks {:.2f} mins.\n'.format((time.time() - start) / 60))
        return True

    def setup_parser(self):
        """
        Sets up an argument parser
        """
        parser = argparse.ArgumentParser(description='ocr recognition')

        parser.add_argument('--image_path', help='path to image_folder or image_file which contains text images')
        parser.add_argument('--workers', type=int, help='number of data loading workers', default=4)
        parser.add_argument('--batch_size', type=int, default=1, help='input batch size')
        parser.add_argument('--saved_model', help="path to saved_model to evaluation")
        parser.add_argument('--logfilepath', help="path to log to demo")

        """ Data processing """
        parser.add_argument('--batch_max_length', type=int, default=25, help='maximum-label-length')
        parser.add_argument('--imgH', type=int, default=32, help='the height of the input image')
        parser.add_argument('--imgW', type=int, default=100, help='the width of the input image')
        parser.add_argument('--rgb', action='store_true', help='use rgb input')
        parser.add_argument('--character', type=str, default='0123456789abcdefghijklmnopqrstuvwxyz',
                            help='character label')
        parser.add_argument('--sensitive', action='store_true', help='for sensitive character mode')
        parser.add_argument('--PAD', action='store_true', help='whether to keep ratio then pad for image resize')
        """ Model Architecture """
        parser.add_argument('--num_fiducial', type=int, default=20, help='number of fiducial points of TPS-STN')
        parser.add_argument('--input_channel', type=int, default=1,
                            help='the number of input channel of Feature extractor')
        parser.add_argument('--output_channel', type=int, default=512,
                            help='the number of output channel of Feature extractor')
        parser.add_argument('--hidden_size', type=int, default=256, help='the size of the LSTM hidden state')


        return parser



    def apply(self, image, timestamp):

        pred, timestamp = detect_ocr(self, image, timestamp)
        return pred, timestamp
        
