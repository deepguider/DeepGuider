# module 내에 opencv가 포함이 되어있어야해..
# input  : timestamp , image
# output : <double>angle (unit : radian)
import cv2
import square2
import module_roaddir as mr
from module_roaddir import test_class as test_c

print(square2.base)
print(square2.square(10))

print('hello')

john = mr.Person('존',28,'대전유성구신성동')
john.greeting()

print('check')

sample = test_c('sample.png')
print(sample.get_angle)
sample.show_size()
sample.show_pic()
