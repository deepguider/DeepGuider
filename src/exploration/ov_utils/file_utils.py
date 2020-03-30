# -*- coding: utf-8 -*-
import os
import numpy as np
import cv2


# borrowed from https://github.com/lengstrom/fast-style-transfer/blob/master/src/utils.py
def get_files(img_dir):
    image_list, sf_list, bbox_list, depth_list = list_files(img_dir)
    return image_list, sf_list, bbox_list, depth_list

# gwanjung dataset
def list_files(in_path):
    img_files = []
    sf_files = []
    bbox_files = []
    depth_list = []
    for (dirpath, dirnames, filenames) in os.walk(in_path):
        for file in filenames:
            filename, ext = os.path.splitext(file)
            ext = str.lower(ext)
            if ext == '.bmp':
                img_files.append(os.path.join(dirpath, file))
            # if ext == '.jpg' or ext == '.jpeg' or ext == '.gif' or ext == '.png' or ext == '.pgm':
            elif "sf" in filename:
                sf_files.append(os.path.join(dirpath, file))
            elif ext == '.xml' or ext == '.gt' or ext == '.txt':
                bbox_files.append(os.path.join(dirpath, file))
            elif "depth" in filename:
                depth_list.append(os.path.join(dirpath, file))
            elif ext == '.zip':
                continue
    img_files.sort()
    sf_files.sort()
    bbox_files.sort()
    depth_list.sort()
    return img_files, sf_files, bbox_files, depth_list

# Starbucks dataset
# def list_files(in_path):
#     img_files = []
#     sf_files = []
#     bbox_files = []
#     depth_list = []
#     for (dirpath, dirnames, filenames) in os.walk(in_path):
#         for file in filenames:
#             filename, ext = os.path.splitext(file)
#             ext = str.lower(ext)
#             if ext == '.bmp':
#                 img_files.append(os.path.join(dirpath, file))
#             # if ext == '.jpg' or ext == '.jpeg' or ext == '.gif' or ext == '.png' or ext == '.pgm':
#             elif "sf" in filename:
#                 sf_files.append(os.path.join(dirpath, file))
#             elif ext == '.xml' or ext == '.gt' or ext == '.txt':
#                 bbox_files.append(os.path.join(dirpath, file))
#             elif "depth" in filename:
#                 depth_list.append(os.path.join(dirpath, file))
#             elif ext == '.zip':
#                 continue
#     img_files.sort()
#     sf_files.sort()
#     bbox_files.sort()
#     depth_list.sort()
#     return img_files, sf_files, bbox_files, depth_list


def saveResult(img_file, img, boxes, dirname='./result/', verticals=None, texts=None):
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
        filename, file_ext = os.path.splitext(os.path.basename(img_file))
        dirname2, file_ext = os.path.splitext("/".join(img_file.split("/")[7:-1]))
        # result directory
        res_file = dirname + dirname2 + "/res_" + filename + '.txt'
        res_img_file = dirname + dirname2 + "/res_" + filename + '.jpg'

        if not os.path.isdir(dirname + dirname2):
            os.mkdir(dirname + dirname2)
        #
        with open(res_file, 'w') as f:
            for i, box in enumerate(boxes):
                poly = np.array(box).astype(np.int32).reshape((-1))
                strResult = ','.join([str(p) for p in poly]) + '\r\n'
                f.write(strResult)
        #
                poly = poly.reshape(-1, 2)
                cv2.polylines(img, [poly.reshape((-1, 1, 2))], True, color=(0, 0, 255), thickness=2)
                ptColor = (0, 255, 255)
                if verticals is not None:
                    if verticals[i]:
                        ptColor = (255, 0, 0)

                if texts is not None:
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    cv2.putText(img, "{}".format(texts[i]), (poly[0][0]+1, poly[0][1]+1), font, font_scale, (0, 0, 0), thickness=1)
                    cv2.putText(img, "{}".format(texts[i]), tuple(poly[0]), font, font_scale, (0, 255, 255), thickness=1)

        # Save result image
        cv2.imwrite(res_img_file, img)


def get_annos(anno_dir, data_name="test"):
    anno_file = anno_dir + data_name + '.txt'
    im_paths = []
    target_pois = []
    with open(anno_file, 'r') as f:
        targets = f.readlines()
    for tt in targets:
        im_path, target_poi = tt.rstrip().split(" ")
        im_paths.append(im_path)
        target_pois.append(target_poi)
    return im_paths, target_pois


def get_templates(data_dir, targetPOI):
    if targetPOI == "PARIS_BAGUETTE":
        template_list = [
            data_dir + "logo/p1.jpg",
            data_dir + "logo/p2.jpg",
            data_dir + "logo/p3.jpg",
            data_dir + "logo/p4.jpg",
            data_dir + "logo/p5.jpg",
            data_dir + "logo/p6.jpg",
        ]
        main_template = data_dir + "logo/p1.jpg"
    elif targetPOI == "PASCUCCI":
        template_list = [
            data_dir + "logo/cp1.jpg",
            data_dir + "logo/cp2.jpg",
            data_dir + "logo/cp3.jpg",
            data_dir + "logo/cp4.jpg",
            data_dir + "logo/cp5.jpg",
        ]
        main_template = data_dir + "logo/cp1.jpg"
    elif targetPOI == "CU":
        template_list = [
            # data_dir + "logo/cu1.jpg",
            data_dir + "logo/cu2.jpg",
            data_dir + "logo/cu3.jpg",
        ]
        main_template = data_dir + "logo/cu2.jpg"
    elif targetPOI == "FRANGCORS_FANCY":
        template_list = [
            data_dir + "logo/ff1.jpg",
            data_dir + "logo/ff2.jpg",
            data_dir + "logo/ff3.jpg",
        ]
        main_template = data_dir + "logo/ff1.jpg"
    elif targetPOI == "HUE_GIMBAB":
        template_list = [
            data_dir + "logo/h1.jpg",
            data_dir + "logo/h2.jpg",
            data_dir + "logo/h3.jpg",
            data_dir + "logo/h4.jpg",
        ]
        main_template = data_dir + "logo/h1.jpg"
    elif targetPOI == "LOTTERIA":
        template_list = [
            data_dir + "logo/l1.jpg",
            data_dir + "logo/l2.jpg",
            data_dir + "logo/l3.jpg",
            data_dir + "logo/l4.jpg",
            data_dir + "logo/l5.jpg",
        ]
        main_template = data_dir + "logo/l1.jpg"
    elif targetPOI == "LOTTE_TOUR":
        template_list = [
            data_dir + "logo/lg1.jpg",
            data_dir + "logo/lg2.jpg",
            data_dir + "logo/lg3.jpg",
        ]
        main_template = data_dir + "logo/lg1.jpg"
    else:
        print('There is no POI named %s'%(targetPOI))
        raise IOError

    templates = []
    for template in template_list:
        template = cv2.imread(template)
        # template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        templates.append(template)
    main_template = cv2.imread(main_template)
    return templates, main_template


def get_gt_templates(targetPOI):
    if targetPOI == "PARIS_BAGUETTE":
        optimal_viewpoint = "11-3-08"
    elif targetPOI == "PASCUCCI":
        optimal_viewpoint = "06-3-08"
    elif targetPOI == "CU":
        optimal_viewpoint = "32-3-08" #7%
    elif targetPOI == "FRANGCORS_FANCY":
        optimal_viewpoint = "41-3-08" #8%
    elif targetPOI == "HUE_GIMBAB":
        optimal_viewpoint = "28-3-08" #8%
    elif targetPOI == "LOTTERIA":
        optimal_viewpoint = "19-3-08" #8%
    elif targetPOI == "LOTTE_TOUR":
        optimal_viewpoint = "00-0-08" #12.8%
    else:
        print('There is no POI named %s'%(targetPOI))
        raise IOError

    return optimal_viewpoint