# input  : image, timestamp
# output : <double>angle (unit : radian), <double>prob

import cv2
from road_direction_recognizer import RoadDirectionRecognizer as rd

timestamp = 123.456
filename = 'sample.jpg'

module = rd()
image = cv2.imread(filename)
module.apply(image, timestamp)
