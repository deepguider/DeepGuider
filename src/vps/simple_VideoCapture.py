import cv2
import numpy as np
import os
import glob
import imutils  # pip install imutils
import argparse

class simple_VideoCapture():
    def __init__(self, img_dir, ext="jpg"):
        self.img_dir = img_dir
        self.ext = ext

        self.img_list = glob.glob(os.path.join(self.img_dir, "*." + self.ext))
        self.img_list.sort()
        self.frame_length = len(self.img_list)
        self.frame_idx = 0

    def set(self, param, frame_idx):
        if param == cv2.CV_CAP_PROP_POS_FRAMES:
            self.frame_idx = frame_idx

    def get(self, param):
        if param == cv2.CV_CAP_PROP_POS_FRAMES:
            return self.frame_idx
        return 0

    def read(self):
        ret, frame = False, None
        if (self.frame_idx >= 0) and (self.frame_idx < self.frame_length):
            fname = self.img_list[self.frame_idx]
            frame = cv2.imread(fname)
            self.frame_idx+=1
            ret = True
        return ret, frame

    def get_path(self):
        if (self.frame_idx > 0) and (self.frame_idx < self.frame_length):
            fname = self.img_list[self.frame_idx-1]  # return previous file path which is current frame's path
        return fname

    def release(self):
        return 0

if __name__ == "__main__":
    cap = simple_VideoCapture("./img", "jpg") 
    cnt = 0
    while(True):
        ret, frame = cap.read()    # Read 결과와 frame
        if not ret:
            break
        cv2.imshow('color', frame)    # 컬러 화면 출력        cv2.imshow('frame_gray', gray)    # Gray 화면 출력
        cnt += 1
        print("Reading {}           \r".format(cnt), end='')
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
