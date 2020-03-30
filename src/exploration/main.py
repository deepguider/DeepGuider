#!/usr/bin/env python

from src.exploration.active_navigation import ActiveNavigationModule
import argparse
import src.exploration.ov_utils.file_utils as file_utils
from src.exploration.ov_utils.navi_data import Navi
from src.exploration.ov_utils.myutils import get_img
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

# # load img + action trajectory
data_list = [os.path.join('./data_exp/img_trajectory', x) for x in os.listdir('./data_exp/img_trajectory')]
data = joblib.load(np.random.choice(data_list))

img_list = data['rgb']
guidance_list = data['action']

vis_mem = []
for i in range(len(img_list)):
    anm.encodeVisualMemory(Image.fromarray(img_list[i]), guidance_list[i], None, test_mode=True)
    vis_mem.append(anm.vis_mem)

anm.vis_mem = torch.cat(vis_mem, 0)
anm.enable_recovery = True
if anm.isRecoveryGuidanceEnabled():
    curr_img = Image.fromarray(img_list[np.random.randint(len(img_list))])
    anm.calcRecoveryGuidance(state='lost', img=curr_img)
    recovery_guidance = anm.recovery_guidance
    print('Recovery guidance from the last inserted visual memory : ', recovery_guidance)
# return anm.getRecoveryGuidance()
# if anm.isExplorationGuidanceEnabled():
#   return anm.getExplorationGuidance()
    
# anm.calcNeedForOptimalViewpointGuidance(topometric_pose_conf, poi_conf, entrance, entrance_conf, poi_successes)


if anm.isOptimalViewpointGuidanceEnabled():
    image_list, sf_list, bbox_list, depth_list = file_utils.get_files(args.data_folder)
    im_paths, target_pois = file_utils.get_annos(args.data_folder + 'anno/')
    im_cnt = 0
    tot_acc = []
    tot_test = len(im_paths)
    for im_path, target_poi in zip(im_paths, target_pois):
        im_cnt += 1
        file = im_path
        init_view = file
        NV.file2curpos(file)
        if args.verbose:
            print("Testing... {}/{}, Target: {}".format(im_cnt, tot_test, target_poi))
        gt_view = file_utils.get_gt_templates(target_poi)
        anm.calcOptimalViewpointGuidance(im_path, target_poi)

        c_guidance = anm.getCentralViewpointGuidance()
        central_view = anm.getCentralViewpointPath()
        guidance = anm.getOptimalViewpointGuidance()
        final_view = anm.getOptimalViewpointPath()
        initial_img = get_img(args, init_view)
        central_img = get_img(args, central_view)
        final_img = get_img(args, final_view)

        fig = plt.figure(figsize=(6, 10))
        ax = plt.subplot(311)
        ax.imshow(np.uint8(initial_img))
        ax.set_title("An initial image [Target: {}] \n(Guidance: Rotate {} degree)".format(target_poi, c_guidance[-1]), fontsize='medium')
        ax.axis('off')

        ax = plt.subplot(312)
        ax.imshow(np.uint8(central_img))
        ax.set_title("A rotated image for locating the target logo in the center \n(Guidance: Move %.2fm on WE-axis and %.2fm on NS-axis then Rotate %d degree)"% (guidance[0], guidance[1], guidance[2]), fontsize='medium')
        ax.axis('off')

        ax = plt.subplot(313)
        ax.imshow(np.uint8(final_img))
        ax.set_title("A final view according to the guidance", fontsize='medium')
        ax.axis('off')

        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())
        fig.savefig('./results/ov_test_{}.png'.format(im_cnt))
        plt.close()
        if args.verbose:
            print("Central guidance [rot]): ", c_guidance[-1])
            print("Optimal guidance [disp_x, disp_y, rot]: [%.2fm, %.2fm, %d]" % (guidance[0], guidance[1], guidance[2]))