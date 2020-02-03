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


class POIRecognizer:
    def __init__(self):
        self.sim_threshold = 0.90
        self.output_txt = 'out.txt'

    def initialize(self):
        self.sim_threshold = 0.90
        self.output_txt = 'out.txt'

        print('PythonTest: Initialization done...!\n')
        print(sys.version)
        return True

    def apply(self, image, timestamp):
        print('PythonTest: Apply...!\n')
        return self.sim_threshold, self.output_txt
        
