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
        self.value2 = 1.2
        return True

    def test_imgserver(self):
        ## Test for Img Server Begin
        import get_streetview
        outdir='./download_jpg'
        get_streetview.makedir(outdir)
        try:
            get_streetview.GetStreetView(gps_lat=36.3851418, gps_long=127.3768362,
                roi_radius=100, ipaddr='localhost', server_type="streetview",
                req_type="wgs",outdir=outdir)
        except:
            print("Image server is not available.")

    def apply(self, image, timestamp):
        print('PythonTest: Apply...!\n')
        self.test_imgserver()
        return self.value1, self.value2
        
