#!/usr/bin/env python
import numpy as np 
# import tensorflow as tf
from ov_utils.myutils import make_mask, template_matching, get_surfacenormal, get_bbox, get_depth, get_img
from ov_utils.config import normal_vector
import ov_utils.file_utils as file_utils
import eVM_utils.utils as eVM_utils
from eVM_utils.eVM_model import encodeVisualMemory
from recovery_policy import Recovery
import torch
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import os
import joblib
import cv2
from PIL import Image
import ov_utils.file_utils as file_utils

"""
Active Navigation Module:
A module that maintains visual memory and provides three kinds of active navgitation guidance;
(1) recovery guidance, (2) exploration guidance, (3) optimal viewpoint guidance.

Author
------
Yunho Choi, Obin Kwon, Nuri Kim, Hwiyeon Yoo
"""

class ActiveNavigationModule():
    """Active Navigation Module"""
    def __init__(self):
        # map_manager = MapManager()
        # self.map = map_manager.getMap()
        # self.args = args
        self.list2encode = []
        self.vis_mem = []
        self.vis_mem_encoder = encodeVisualMemory()
        self.vis_mem_encoder_model = None
        
        self.enable_recovery = False
        self.recovery_policy = Recovery()
        self.recovery_guidance = None
        
        self.enable_exploration = False
        self.exploration_policy = None  
        self.exploration_guidance = None

        self.enable_ove = False
        self.optimal_viewpoint_guidance = [0, 0, 0]

        data_list = [os.path.join('./data_exp/img_trajectory', x) for x in os.listdir('./data_exp/img_trajectory')]
        data = joblib.load(np.random.choice(data_list))
        self.img_list = data['rgb']
        self.guidance_list = data['action']

        self.ove_data_folder = './data_exp/optimal_viewpoint/'
        image_list, sf_list, bbox_list, depth_list = file_utils.get_files(self.ove_data_folder)
        self.im_paths, self.target_pois = file_utils.get_annos(self.ove_data_folder + 'anno/')

    def initialize(self):
        
        try:
            self.vis_mem_encoder.load_state_dict(torch.load(self.vis_mem_encoder_model))
        except:
            print("Cannot load pretrained encodeVisualMemory model")
            pass

        self.enable_recovery = True
        self.enable_ove = True

        

    def encodeVisualMemory(self, img, guidance, random_action=False, flush=False, exp_active=False):
        """
        Visual Memory Encoder Submodule:
        A module running consistently which encodes visual trajectory information from the previous node to the current location.
        When reaching new node, flush the memory

        Input:
        - topometric_pose: node_id, edge_idx, dist
        - guidance from previous node 
        - img

        Output:
        - visual memory: publish with message type VisMem

        Dependency:
        - Localizer
        """

        if random_action: # For test
            test_act = np.random.randint(0, 3)
            onehot_test_act = np.zeros(3)
            onehot_test_act[test_act] = 1
            tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
            tensor_action = torch.tensor(onehot_test_act, dtype=torch.float32).unsqueeze(0)
            self.list2encode.append([img, onehot_test_act])
            vis_mem_seg, _ = self.vis_mem_encoder(tensor_img, tensor_action)
            self.vis_mem.append(vis_mem_seg)

        else:
            # flush when reaching new node
            if flush is True: # topometric_pose.dist < 0.1:
                self.list2encode = []
                self.vis_mem = []

            elif exp_active is False:
                action = np.zeros(3)
                action[guidance] = 1
                self.list2encode.append([img,action])
                try:
                    tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
                    tensor_action = torch.tensor(action).unsqueeze(0)
                    vis_mem_seg, _ = self.vis_mem_encoder(tensor_img, tensor_action)
                    self.vis_mem.append(vis_mem_seg)
                except:
                    print("NotImplementedError")
                    self.vis_mem = None

    def calcRecoveryGuidance(self, img=None):
        """
        Recovery Guide Provider Submodule:
        A module that guides a robot to return to the previous node, when StateDeterminant module
        determines that the robot is lost.
        If matching the retrieved image with the visual memory fails, call Exploration Guidance Module instead.

        Input:
        # - state: state from StateDeterminant Module
        - img: curreunt image input (doesn't need if visual memory contains the current input image)
        Output:
        - action(s) guides to reach previous POI
        """

        if self.enable_exploration is False:
            self.enable_recovery = True

        if self.enable_recovery is True:
            try:
                # calculate the actions to return to the starting point of visual memory # doesn't need pose
                if img is None and len(self.list2encode) > 0:
                    img = self.list2encode[-1][0]
                elif img is None:
                    print('Nothing to encode or calculate because there was no input at all')
                    raise Exception
                # encode the input image
                test_act = np.random.randint(0, 3)
                onehot_test_act = np.zeros(3)
                onehot_test_act[test_act] = 1
                tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
                tensor_action = torch.tensor(onehot_test_act, dtype=torch.float32).unsqueeze(0)
                _, img_feature = self.vis_mem_encoder(tensor_img, tensor_action)
                # done: is back home, info: whether visual memory matching succeeded or not
                actions, done, info = self.recovery_policy(self.vis_mem, img_feature)
            except:
                raise
                # TODO: if recovery_policy fails to calculate the recovery actions,
                #       just reverse the actions in the visual memory (using self.list2encode)
                #       - can't implement now due to the ambiguity of the action space
                print("NotImplementedError")
                actions, done, info = [0., -0.40, 0.], False, False

            self.recovery_guidance = actions

        if info is False:
            self.enable_recovery, self.enable_exploration = False, True

        if done is True:
            self.enable_recovery, self.enable_exploration = True, False


    def calcExplorationGuidance(self, img):
        """
        Exploration Guidance Provider Submodule:
        A module that guides the robot to reach nearby POI using nearby visual information and visual memory.
        This module is triggered when the POI Recovery Guidance Module judges that it is hard to 
        return to a node associated with the previous POI, based on the visual memory matching result.

        Input:
        # - state: state from StateDeterminant Module
        ---(topometric_pose_conf: confidence of topometric pose)
        - (tentative) POI detection result
        - img: curreunt image input
        - visual memory

        Output:
        - action(s) guides to reach nearby POI

        Dependency:
        - Localizer module
        """
        # if state == 'normal':
        #     self.enable_exploration = False

        if self.enable_exploration is True:
            try:
                actions, done, info = self.exploration_policy(img, self.vis_mem)
            except:
                print("NotImplementedError")
                actions, done, info = [0., 0.40, 0.], False, False # ['forward','forward','forward']

            self.exploration_guidance = actions
            if done is True:
                self.enable_recovery, self.enable_exploration = True, False
        else:
            self.exploration_guidance = None
 

    # def calcNeedForOptimalViewpointGuidance(self, topometric_pose, poi_conf, entrance, entrance_conf, poi_successes):
    #     """
    #     A module calculates the need for optimal viewpoint guidance.
    #     It considers the results of POI detection and current location (destination, door).

    #     Input:
    #     - topometric_pose: node_id, edge-idx, dist
    #     - poi_conf: detection results of POI (confidence score)
    #     - (tentative) entrance: If there is an building entrance near current location, True, else False (Boolean) , tentative due to the API ambiguity.
    #     - entrance_conf: detection results of building entrances 
    #     - poi_successes: A sequence of POI detection result (confidence score)

    #     Output:
    #     - require the optimal viewpoint guidance optimization (Boolean)
    #     """

    #     # Cases
    #     case1 = np.mean(poi_successes) < 0.5
    #     case2 = ((self.map.getNode(topometric_pose.node_id).edges[topometric_pose.edge_idx].length - topometric_pose.dist) < 0.2) and (poi_conf < 0.5)
    #     case3 = entrance and (entrance_conf < 0.5)

    #     if case1 or case2 or case3:
    #         self.enable_ove = True
    #     else:
    #         self.enable_ove = False

    def calcOptimalViewpointGuidance(self, img_path, target_poi):
        """
        Optimal Viewpoint Guidance Provider Submodule:
        A module that provides an optimal viewpoint guidance for enhancing POI detection.

        Input:
        - img: current image input
        - target_poi: target PoI (searching object)

        Output:
        - Action(s) guides to find an optimal viewpoint
        """

        if self.enable_ove:
            self.ov_guidance = self.optimal_viewpoint(img_path, target_poi)

    def optimal_viewpoint(self, file_path=None, target_poi=None):
        if file_path and target_poi is None:
            file_path, target_poi = self.im_paths[0], self.target_pois[0]
        heading1 = D1 = heading2 = 0
        templates, main_template, opt_ratio = file_utils.get_templates(self.ove_data_folder, targetPOI=target_poi)
        img = get_img(self.ove_data_folder, file_path)
        depth_ = get_depth(self.ove_data_folder, file_path)
        bbox = get_bbox(self.ove_data_folder, file_path)
        h, w, _ = img.shape

        if len(bbox) > 0:
            bbs = []
            for bb in bbox:
                bbs.append(bb.rstrip().split(','))
            bbs = np.stack([np.float32(bbs[i]) for i in range(len(bbs))])
            bbs = np.reshape(bbs, [-1, 4, 2])
            bbs = bbs.astype(np.int32)

            # Find the bounding box for the target POI
            template_matched, bbox, index, _ = template_matching(img, bbs, templates, main_template)
            bbox = np.reshape(bbox, [-1, 4, 2])
            mask = make_mask(bbox, shape=[h, w])
            if np.sum(mask) > 0 and template_matched:
                depth = depth_[mask == 1]
                center_depth = np.mean(depth_[110:140, 235:265], (0,1))

                if np.mean(depth) <= 0.01 and len(bbs) > 1:
                    indices = list(np.arange(len(bbs)))
                    indices.pop(index)
                    bbs = bbs[indices]
                    bbs = np.reshape(bbs, [-1, 4, 2])
                    template_matched, bbox, _, _ = template_matching(img, bbs, templates, main_template)
                    bbox = np.reshape(bbox, [-1, 4, 2])
                    mask = make_mask(bbox, shape=[h, w])
                    depth = depth_[mask == 1]

                if np.mean(depth) <= 0.01:
                    depth = depth + 0.01

                if np.mean(center_depth) <= 0.01:
                    center_depth += 0.01

                # TODO: Estimate the exact distance
                D = np.mean(depth) * 19.2  # d1
                center_D = np.mean(center_depth) * 19.2  # d2

                # Decide the amount of the movement using depth
                ratio = (abs(bbox[0, 3, 1] - bbox[0, 0, 1]) + abs(bbox[0, 1, 1] - bbox[0, 2, 1])) / 2 / h

                # Decide the moving direction
                sf = get_surfacenormal(self.ove_data_folder, file_path)
                sf_norm = np.mean(sf[mask == 1], 0)
                sf_norm = sf_norm * 2 - 1
                sf_norm = sf_norm / np.linalg.norm(sf_norm, 2)

                POI_centloc = ("left", "right")[(bbox[0, 2, 0] + bbox[0, 0, 0]) / 2 > w / 2] #Left: Logos on the left from center, Right: Logos on the right from center

                # Rotate the agent until the POI is locate on the center
                center_sf_norm = np.mean(sf[110:140, 235:265], (0, 1))
                center_sf_norm = center_sf_norm * 2 - 1
                center_sf_norm = center_sf_norm / np.linalg.norm(center_sf_norm, 2)

                # Check that the bounding box is on the left or right buildings (not street or sky)
                if abs(sf_norm[1]) < 0.8:
                    center_poi_theta = np.arccos(np.dot(sf_norm, center_sf_norm))

                    # Align the POI and the camera center
                    heading = 180 / np.pi * center_poi_theta
                    if POI_centloc == "left":
                        heading = -heading

                    if abs(heading) > 15:
                        return [0, 0, heading]

                theta_ = np.arccos(np.dot(center_sf_norm, normal_vector)) # center point
                theta_tilde = np.arctan(D*np.tan(theta_)/np.abs(center_D-D))
                cond = (center_D > D)
                theta = (theta_ + theta_tilde - np.pi/2) if cond else (theta_ + np.pi/2 - theta_tilde)
                D = D / np.sin(theta_tilde) # distance between the robot and signage, not depth of the signage
                D0 = D * (1 - np.maximum(ratio / opt_ratio, 0.95))
                thetad = np.arctan(((D - D0) * np.sin(theta)) / (D - (D - D0) * np.cos(theta)))

                #Rotate before going straight
                if cond:
                    heading1 = 180 / np.pi * (thetad - (np.pi/2 - theta_tilde)) 
                    heading1 = (heading1, -heading1)[POI_centloc == "right"]
                else:
                    heading1 = 180 / np.pi * (thetad + (np.pi/2 - theta_tilde))
                    heading1 = (-heading1, heading1)[POI_centloc == "right"]
                D1 = ((D - D0)*np.sin(theta)/(np.sin(thetad)), D - D0)[thetad == 0] # Sine Law

                # Rotate to see the POI
                if cond:
                    heading2 = 180 / np.pi * (theta + thetad)
                    heading2 = (-abs(heading2), abs(heading2))[POI_centloc == "right"]
                else:
                    heading2 = 180 / np.pi * (theta + thetad)
                    heading2 = (abs(heading2), -abs(heading2))[POI_centloc == "right"]

        return [heading1, D1, heading2]

    def getVisualMemory(self):
        return self.vis_mem

    def isRecoveryGuidanceEnabled(self):
        return self.enable_recovery

    def isExplorationGuidanceEnabled(self):
        return self.enable_exploration

    def isOptimalViewpointGuidanceEnabled(self):
        return self.enable_ove

    def getExplorationGuidance(self, img, guidance, flush, exp_active, im_path=None, target_poi=None):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        curr_img = Image.fromarray(img)
        self.encodeVisualMemory(curr_img, guidance, flush=flush, exp_active=exp_active)

        if exp_active is True:
            if self.isOptimalViewpointGuidanceEnabled() and target_poi is not None:
                    self.calcOptimalViewpointGuidance(im_path, target_poi)
                    guidance = self.ov_guidance
                    print("Optimal Guidance [theta1, d, theta2]: [%.2f degree, %.2fm, %.2f degree]" % (guidance[0], guidance[1], guidance[2]))
                    return [guidance], 'OptimalViewpoint'

            if self.isRecoveryGuidanceEnabled():
                self.calcRecoveryGuidance(img=curr_img)
                print('Recovery guidance from the last inserted visual memory : ', self.recovery_guidance)

                if self.enable_exploration is False:
                    return [self.recovery_guidance], 'Recovery'

            if self.isExplorationGuidanceEnabled():
                self.calcExplorationGuidance(img=curr_img)
                print('Exploration guidance from the last inserted visual memory : ', self.exploration_guidance)
                return [self.exploration_guidance], 'Exploration'
        else:
            return [[0., 0., 0.]], 'Normal'


    # def getRecoveryGuidance(self):
    #     if self.enable_recovery:
    #         return self.recovery_guidance
    #     else:
    #         return None

    # def getExplorationGuidance(self):
    #     if self.enable_exploration:
    #         return self.exploration_guidance
    #     else:
    #         return None

    # def getOptimalViewpointGuidance(self):
    #     if self.enable_ove:
    #         return self.ov_guidance
    #     else:
    #         return None

# if __name__ == "__main__":
#   anm = ActiveNavigationModule()