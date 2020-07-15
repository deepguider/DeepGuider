# -*- coding: utf-8 -*-
import os
import numpy as np
import cv2
from PIL import Image
from craft import imgproc

# borrowed from https://github.com/lengstrom/fast-style-transfer/blob/master/src/utils.py
def get_files(img_dir):
    imgs, masks, xmls = list_files(img_dir)
    return imgs, masks, xmls

def list_files(in_path):
    img_files = []
    mask_files = []
    gt_files = []
    for (dirpath, dirnames, filenames) in os.walk(in_path):
        for file in filenames:
            filename, ext = os.path.splitext(file)
            ext = str.lower(ext)
            if ext == '.jpg' or ext == '.jpeg' or ext == '.gif' or ext == '.png' or ext == '.pgm':
                img_files.append(os.path.join(dirpath, file))
            elif ext == '.bmp':
                mask_files.append(os.path.join(dirpath, file))
            elif ext == '.xml' or ext == '.gt' or ext == '.txt':
                gt_files.append(os.path.join(dirpath, file))
            elif ext == '.zip':
                continue
    # img_files.sort()
    # mask_files.sort()
    # gt_files.sort()
    return img_files, mask_files, gt_files


def img_crop(img, coordinate):

    img_mono= Image.fromarray(img).convert('L')

    x1 = float(coordinate.split(',')[0])
    y1 = float(coordinate.split(',')[1])
    x2 = float(coordinate.split(',')[2])
    y2 = float(coordinate.split(',')[3])
    x3 = float(coordinate.split(',')[4])
    y3 = float(coordinate.split(',')[5])
    x4 = float(coordinate.split(',')[6])
    y4 = float(coordinate.split(',')[7])

    x_min = max(min(x1, x2, x3, x4),0.0)
    x_max = max(x1, x2, x3, x4)

    y_min = max(min(y1, y2, y3, y4),0.0)
    y_max = max(y1, y2, y3, y4)

    crop_position = (x_min, y_min, x_max, y_max)
    # print(crop_position)

    imgCrop = img_mono.crop(crop_position)


    return imgCrop,crop_position


def imgcrop(image_for_recog,boxes ):
    detection_list = []
    for i, box in enumerate(boxes):
        poly = np.array(box).astype(np.int32).reshape((-1))
        strResult = ','.join([str(p) for p in poly])

        img_crop_result, crop_position = img_crop(image_for_recog, strResult)
        # detection_list.append([img_crop_result,strResult])
        detection_list.append([img_crop_result, crop_position])
    return detection_list

def saveResult(img_file, img, image_for_recog, boxes, dirname='./data_ocr/', verticals=None, texts=None):
        """ save text detection result one by one
        Args:
            img_file (str): image file name
            img (array): raw image context
            boxes (array): array of result file
                Shape: [num_detections, 4] for BB output / [num_detections, 4] for QUAD output
        Return:
            None
        """
        img = np.array(img)

        # make result file list
        res_file = dirname + 'res.txt'
        res_img_file = dirname + 'res.jpg'

        if type(img_file) is str:
            filename, file_ext = os.path.splitext(os.path.basename(img_file))

            # result directory
            res_file = dirname + "res_" + filename + '.txt'
            res_img_file = dirname + "res_" + filename + '.jpg'


        if not os.path.isdir(dirname):
            os.mkdir(dirname)

        detection_list = []

        # with open(res_file, 'w') as f:
        for i, box in enumerate(boxes):
                poly = np.array(box).astype(np.int32).reshape((-1))
                # strResult = ','.join([str(p) for p in poly]) + '\r\n'
                strResult = ','.join([str(p) for p in poly])

                # print(strResult)
                # f.write(strResult)

                #send info to regonition part  position , and image array

                img_crop_result, crop_position = img_crop(image_for_recog,strResult)
                # detection_list.append([img_crop_result,strResult])
                detection_list.append([img_crop_result,crop_position])

                poly = poly.reshape(-1, 2)
                cv2.polylines(img, [poly.reshape((-1, 1, 2))], True, color=(0, 0, 255), thickness=2)
                ptColor = (0, 255, 255)
                if verticals is not None:
                    if verticals[i]:
                        ptColor = (255, 0, 0)


                if texts is not None:

                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    # cv2.putText(img, "{}".format(texts[i]), (poly[0][0]+1, poly[0][1]+1), font, font_scale, (0, 0, 0), thickness=1)
                    # cv2.putText(img, "{}".format(texts[i]), tuple(poly[0]), font, font_scale, (0, 255, 255), thickness=1)
                    cv2.putText(img, "{}".format('a'), (poly[0][0] + 1, poly[0][1] + 1), font, font_scale, (0, 0, 0),
                    thickness=1)
                    cv2.putText(img, "{}".format('b'), tuple(poly[0]), font, font_scale, (0, 255, 255), thickness=1)

        # Save result image
        cv2.imwrite(res_img_file, img)
        print('check result : ' + res_img_file)
        return detection_list