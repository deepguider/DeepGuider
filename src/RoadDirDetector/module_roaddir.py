import cv2
from matplotlib import pyplot as plt

test_num = 5

def test_func(n):
    return n*n

class test_class:
    def __init__(self, filename):
        self.filename = filename
        self.sample = cv2.imread(self.filename)
        height,width,channel = self.sample.shape
        self.height = height
        self.width = width
        self.channel = channel

    def show_size(self):
        print('width : ',self.width)
        print('height : ',self.height)
    def show_pic(self):
        sample = cv2.imread(self.filename)
        # h,w,c = sample.shape
        plt.imshow(sample)
        plt.show()
    def get_angle(self):
        return 10

# example
class Person:  # 클래스
    def __init__(self, name, age, address):
        self.name = name
        self.age = age
        self.address = address

    def greeting(self):
        print('안녕하세요. 저는 {0}입니다.'.format(self.name))

