# input  : timestamp , image
# output : <double>angle (unit : radian)

import cv2
import road_dir_recognizer as rdr
from road_dir_recognizer import RoadDirRecognizer as r_dir

time=3.14
filename = 'sample.jpg'

sample = r_dir()
image = cv2.imread(filename)
sample.apply(image,time)
