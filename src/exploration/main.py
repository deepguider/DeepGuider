#!/usr/bin/env python

from active_navigation import ActiveNavigationModule
import argparse
import ov_utils.file_utils as file_utils
import numpy as np
import joblib
import os
from PIL import Image
import torch
import matplotlib.pyplot as plt
import random

parser = argparse.ArgumentParser()
parser.add_argument('--path_direction', default='homing', type=str)    # following / homing
parser.add_argument('--enable_recovery', default=True, type=bool)
parser.add_argument('--enable_ove', default=False, type=bool)
parser.add_argument('--cuda', default=True, type=bool)
args = parser.parse_args('')

if torch.cuda.is_available():
    if args.cuda:
        torch.set_default_tensor_type('torch.cuda.FloatTensor')
    if not args.cuda:
        torch.set_default_tensor_type('torch.FloatTensor')

path_direction = args.path_direction

# Test recovery module
# load img + action trajectory
data_list = [os.path.join('./data_exp/img_trajectory/{}'.format(path_direction), x) for x in os.listdir('./data_exp/img_trajectory/{}'.format(path_direction))]
data = joblib.load(np.random.choice(data_list))

img_list = data['rgb']
guidance_list = data['action']

if path_direction == 'homing':
    img_list = img_list[::1]
    guidance_list = guidance_list[:-1][::-1]
    guidance_list.append(random.randrange(1,4))
#Note that the last action in guidance_list is dummy action

anm = ActiveNavigationModule(args)

for i in range(len(img_list)):
    img_list[i] = Image.fromarray(img_list[i])
    anm.encodeVisualMemory(img_list[i], guidance_list[i])

anm.enable_recovery = args.enable_recovery
# anm.vis_mem = torch.cat(anm.vis_mem, 0)
if anm.isRecoveryGuidanceEnabled():
    curr_img = img_list[np.random.randint(len(img_list))] #No follower data for now, just use guidance img for test
    anm.calcRecoveryGuidance(img=curr_img)
    recovery_guidance = anm.recovery_guidance
    print('Recovery guidance from the last inserted visual memory : ', recovery_guidance)

# # if anm.isExplorationGuidanceEnabled():
    
# # anm.calcNeedForOptimalViewpointGuidance(topometric_pose_conf, poi_conf, entrance, entrance_conf, poi_successes)

# Test ove module
ove_data_folder = './data_exp/optimal_viewpoint/'
anm.enable_ove = args.enable_ove
if anm.isOptimalViewpointGuidanceEnabled():
    image_list, sf_list, bbox_list, depth_list = file_utils.get_files(ove_data_folder)
    im_paths, target_pois = file_utils.get_annos(ove_data_folder + 'anno/')
    im_cnt = 0
    tot_test = len(im_paths)
    for im_path, target_poi in zip(im_paths, target_pois):
        im_cnt += 1
        print("Testing... {}/{}, Target: {}".format(im_cnt, tot_test, target_poi))
        anm.calcOptimalViewpointGuidance(im_path, target_poi)
        guidance = anm.ov_guidance #getOptimalViewpointGuidance()
        print("Optimal Guidance [theta1, d, theta2]: [%.2f degree, %.2fm, %.2f degree]" % (guidance[0], guidance[1], guidance[2]))
anm.enable_ove = False
