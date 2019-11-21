import cv2 as cv

class RoadDirectionRecognizer:
    def __init__(self):
        self.angle = -1   # road direction (radian)
        self.prob = -1    # reliablity of the result. 0: fail ~ 1: success

    def initialize(self):
        self.angle = 0.0  # road direction (radian)
        self.prob = 1.0   # reliablity of the result. 0: fail ~ 1: success
        return True

    def apply(self, image, timestamp):
        self.image = image
        self.timestamp = timestamp

        ##### Process Input #####
        cv.imshow("sample", image)
        cv.waitKey()
        cv.destroyWindow("sample")
        
        ##### Results #####
        self.angle = 10.0
        self.prob = 1.0
        return self.angle, self.prob
