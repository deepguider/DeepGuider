import cv2 as cv

class RoadDirectionRecognizer:
    def __init__(self):
        self.angle = 0.0  # road direction (radian)
        self.prob = 0.0   # reliablity of the result. 0: fail ~ 1: success
        
    def get_param(self):
        # get parameter
        return 0
        
    def set_param(self):
        # set parameter
        return 0
        
    def initialize(self):
        self.angle = 20.0  # road direction (radian)
        self.prob = 2.0   # reliablity of the result. 0: fail ~ 1: success

    def apply(self, image, timestamp):
        self.image = image
        self.timestamp = timestamp

        ##### Process Input #####
        cv.imshow("sample", image)
        cv.waitKey()
        cv.destroyWindow("sample")
        
        ##### Results #####
        self.angle = 10.0
        self.prob = 1
        return self.angle, self.prob

    def getAngle(self):
        return self.angle

    def getProb(self):
        return self.prob
