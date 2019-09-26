# input  : timestamp , image
# output : <double>angle (unit : radian)
import cv2
import module_roaddir as mr
from module_roaddir import test_class as test_c

time=3.14
sample = test_c('sample.png',time)
print(sample.get_angle)
sample.show_size()
sample.show_pic()
