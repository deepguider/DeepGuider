#!/usr/bin/env python

from src.exploration.active_navigation import ActiveNavigationModule
import argparse
import src.exploration.ov_utils.file_utils as file_utils
import numpy as np
from src.exploration.ov_utils.navi_data import Navi

parser = argparse.ArgumentParser(description='Optimal viewpoint estimation')
parser.add_argument('--data_folder', default='./data/optimal_viewpoint/', type=str, help='folder path to input images')
parser.add_argument('--central_guidance', default=True, type=bool, help='A guidance to make a robot to rotate for making PoI on the center of the view')
parser.add_argument('--optimal_guidance', default=True, type=bool, help='A guidance for optimal viewpoint')
parser.add_argument('--enable_ove', default=True, type=bool, help='enable optimal viewpoint estimator')
parser.add_argument('--verbose', default=False, type=bool, help='verbose')
args = parser.parse_args()
NV = Navi()
anm = ActiveNavigationModule(args, NV)


# anm.encodeVisualMemory(img, guidance, topometric_pose)

# anm.calcExplorationGuidance(state, img)

anm.enable_recovery = True
if anm.isRecoveryGuidanceEnabled():
    anm.calcRecoveryGuidance(state='lost')
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
        print("Testing... {}/{}, Target: {}".format(im_cnt, tot_test, target_poi))
        gt_view = file_utils.get_gt_templates(target_poi)
        anm.calcOptimalViewpointGuidance(im_path, target_poi)

        c_guidance = anm.getCentralViewpointGuidance()
        guidance = anm.getOptimalViewpointGuidance()
        final_view = anm.getOptimalViewpointPath()
        if final_view != None:
            print("Central guidance [rot]): ", c_guidance[-1])
            print("Optimal guidance [disp_x, disp_y, rot]: [%.2fm, %.2fm, %d]" % (guidance[0], guidance[1], guidance[2]))
            if NV.dist(final_view, init_view) != 0:
                acc = 1 if NV.dist(final_view, gt_view) < 5 else 0
                tot_acc.append(acc)
        else:
            print(">> No Guidance (Reasons: No detection boxes found, No nearby POI, detection boxes are found on the street)")

    print("Final acc: ", np.mean(tot_acc))
