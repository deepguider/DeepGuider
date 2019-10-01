import cv2 as cv

class RoadDirectionRecognizer:
    def __init__(self):
        self.prob = 0	# reliablity of the result. 0: fail, 1: sucess
        self.angle = 0
    def get_param(self):
        # get parameter
        return 0
    def set_param(self):
        # set parameter
        return 0

    def apply(self, image, timestamp):
        self.timestamp = timestamp
        self.image = image

        ##### Process Input #####
        cv.imshow("sample", image)
        cv.waitKey()
        
        ##### To Do #####
        self.angle = 10.0 # radian
        self.flag = 1 # whether finding road direction success or not
        return self.angle, self.flag

    def getAngle(self):
        return self.angle

    def getProb(self):
        return self.prob
