import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import pickle
import time

from PIL import Image
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGE = True


class PythonTest:
    def __init__(self):
        self.value1 = -1
        self.value2 = -1

    def init_param(self):
        print('PythonTest: Init_param...!\n')
        self.value1 = 0
        self.value2 = 0

    def initialize(self):
        print('PythonTest: Initialization...!\n')
        print(sys.version)
        self.value1 = 0
        self.value2 = 0
        return True

    def apply(self, image, timestamp):
        print('PythonTest: Apply...!\n')
        return self.value1, self.value2
        
