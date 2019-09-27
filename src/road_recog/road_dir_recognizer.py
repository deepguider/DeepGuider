import cv2
from matplotlib import pyplot as plt

class RoadDirRecognizer:
    def __init__(self):
        self.flag = False
        self.angel=0
    def get_param(self):
        # get parameter
        return 0
    def set_param(self):
        # set parameter
        return 0

    def apply(self, image,timestamp):
        self.timestamp = timestamp
        self.image = image
        plt.imshow(self.image)
        plt.show()  # ESC to exit

        ##### To Do #####
        self.angle = 10.0
        self.flag = True # whether finding road direction success or not
        return self.angle, self.flag
