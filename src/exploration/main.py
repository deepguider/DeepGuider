#!/usr/bin/env python

from active_navigation import ActiveNavigationModule
import argparse
import ov_utils.file_utils as file_utils
from ov_utils.navi_data import Navi
import numpy as np
import joblib
import os
from PIL import Image
import torch
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Optimal viewpoint estimation')
parser.add_argument('--data_folder', default='./data_exp/optimal_viewpoint/', type=str, help='folder path to input images')
parser.add_argument('--central_guidance', default=True, type=bool, help='A guidance to make a robot to rotate for making PoI on the center of the view')
parser.add_argument('--optimal_guidance', default=True, type=bool, help='A guidance for optimal viewpoint')
parser.add_argument('--enable_ove', default=True, type=bool, help='enable optimal viewpoint estimator')
parser.add_argument('--verbose', default=False, type=bool, help='verbose')
args = parser.parse_args()
NV = Navi()
anm = ActiveNavigationModule(args, NV)

# Test recovery module
# load img + action trajectory
data_list = [os.path.join('./data_exp/img_trajectory', x) for x in os.listdir('./data_exp/img_trajectory')]
data = joblib.load(np.random.choice(data_list))

img_list = data['rgb']
guidance_list = data['action']

for i in range(len(img_list)):
    anm.encodeVisualMemory(Image.fromarray(img_list[i]), guidance_list[i], None, random_action=True)

anm.enable_recovery = True
anm.vis_mem = torch.cat(anm.vis_mem, 0)
if anm.isRecoveryGuidanceEnabled():
    curr_img = Image.fromarray(img_list[np.random.randint(len(img_list))])
    anm.calcRecoveryGuidance(img=curr_img)
    recovery_guidance = anm.recovery_guidance
    print('Recovery guidance from the last inserted visual memory : ', recovery_guidance)

# # if anm.isExplorationGuidanceEnabled():
# #   return anm.getExplorationGuidance()
    
# # anm.calcNeedForOptimalViewpointGuidance(topometric_pose_conf, poi_conf, entrance, entrance_conf, poi_successes)

# Test ove module
anm.enable_ove = True
if anm.isOptimalViewpointGuidanceEnabled():
    image_list, sf_list, bbox_list, depth_list = file_utils.get_files(args.data_folder)
    im_paths, target_pois = file_utils.get_annos(args.data_folder + 'anno/')
    im_cnt = 0
    tot_test = len(im_paths)
    for im_path, target_poi in zip(im_paths, target_pois):
        im_cnt += 1
        anm.NV.file2curpos(im_path)
        print("Testing... {}/{}, Target: {}".format(im_cnt, tot_test, target_poi))
        anm.calcOptimalViewpointGuidance(im_path, target_poi)
        guidance = anm.getOptimalViewpointGuidance()
        print("Optimal Guidance [disp_x, disp_y, rot]: [%.2fm, %.2fm, %.2f degree]" % (guidance[0], guidance[1], guidance[2]))
anm.enable_ove = False