# input  : image, timestamp
# output : <double>angle (unit : radian), <double>prob

import cv2 as cv
from road_direction_recognizer import RoadDirectionRecognizer as rd

image = cv.imread('road_sample.jpg')
timestamp = 123.456

module = rd()
angle,prob = module.apply(image, timestamp)

print('angle=%f, prob=%f'%(angle,prob))
