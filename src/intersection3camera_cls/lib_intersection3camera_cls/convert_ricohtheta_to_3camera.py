import cv2
import numpy as np
import time
from .modules import panorama as pn
import os
import argparse


def GetPanoTool():
    pano_tool = pn.PanoramicImageTool()
    return pano_tool

def GetPanTiltImg(pano_tool = None, pano_image = None, pan = 0.0, tilt = 0.0, fov = 70.0, height = 960, width = 1280):
    # pano_image is output of cv2.imread()
    # tilt angle in [deg]
    # pan angle in [deg]
    # width and height in [px]

    if pano_tool is None:
        pano_tool = pn.PanoramicImageTool()

    pano_tool.set_panoramic_image(pano_image)
    pano_tool.set_view_angles(pan, tilt, fov)
    crop_image = pano_tool.crop((height, width))
    return crop_image


def imresize(img, scale_percent=60):
    #scale_percent = 60 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def crop_image(frame, pano_tool, pans=[270, 0, 90, 180]):
    '''
    pans = [270, 0, 90, 180]  # front, right, left, back
    '''
    crop_images = []
    for pan in pans:
        crop_image = GetPanTiltImg(pano_tool, frame, pan = pan, tilt = 0.0, fov = 90.0)
        crop_images.append(crop_image)
        # if len(crop_images) == 0:
        #     crop_images = crop_image
        # else:
        #     crop_images = np.hstack([crop_images, crop_image])
    return crop_images


class RicohthetaTo3CameraConverter:
    def __init__(self, facingback=False):
        self.pano_tool = GetPanoTool()
        
        if facingback:
            self.pans=[90, 180, 270]
        else:
            self.pans=[270, 0, 90]

    def convert_ricohtheta_to_3camera(self, ricohtheta_img):
        cropped = crop_image(ricohtheta_img, self.pano_tool, self.pans)

        left_img = cropped[0]
        center_img = cropped[1]
        right_img = cropped[2]
        return left_img, center_img, right_img

#nonintersection_img = cv2.imread("../data_intersection3camera_cls/nonintersection_omni.jpg")
#ricohto3cameraconverter = RicohthetaTo3CameraConverter()
#left, center, right = ricohto3cameraconverter.convert_ricohtheta_to_3camera(nonintersection_img)
#cv2.imshow("left", left)
#cv2.imshow("center", center)
#cv2.imshow("right", right)
cv2.waitKey(0)
