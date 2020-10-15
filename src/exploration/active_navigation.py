#!/usr/bin/env python
import numpy as np 
# import tensorflow as tf
from ov_utils.myutils import make_mask, template_matching, get_surfacenormal, get_bbox, get_depth, get_img
from ov_utils.config import normal_vector
import ov_utils.file_utils as file_utils
import eVM_utils.utils as eVM_utils
from eVM_utils.eVM_model import CNN, encodeVisualMemory, encodeVisualMemoryRelatedPath
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
    def __init__(self, args):
        # map_manager = MapManager()
        # self.map = map_manager.getMap()
        # self.args = args
        self.action_dim = 3
        self.memory_dim = 256
        self.feature_dim = 512
        self.GRU_size = 512
        self.img_size = 224
        self.path_length = 0
        self.list2encode = []
        self.vis_mem = []
        self.feats_along_path = []
        self.rel_pose = None
        self.path_direction = args.path_direction
        self.cuda = args.cuda
        self.encode_im = CNN(self.feature_dim, self.img_size)
        if self.path_direction=='following':
            self.vis_mem_encoder = encodeVisualMemory(self.action_dim, self.memory_dim, self.feature_dim)
        if self.path_direction=='homing':
            self.vis_mem_encoder = encodeVisualMemoryRelatedPath(self.action_dim, self.memory_dim, self.feature_dim)
        self.vis_mem_encoder_model = './data_exp/model/{}/best.pth'.format(self.path_direction)
        
        self.enable_recovery = False
        self.recovery_policy = Recovery(self.action_dim, self.memory_dim, self.feature_dim, self.GRU_size)
        self.recovery_guidance = None
        
        self.enable_exploration = False
        self.exploration_policy = None  
        self.exploration_guidance = None

        self.enable_ove = False
        self.optimal_viewpoint_guidance = [0, 0, 0]

        data_list = [os.path.join('./data_exp/img_trajectory/{}'.format(self.path_direction), x) for x in os.listdir('./data_exp/img_trajectory/{}'.format(self.path_direction))]
        data = joblib.load(np.random.choice(data_list))
        self.img_list = data['rgb']
        self.guidance_list = data['action']
        self.position_list = data['position']
        rotation_quat = data['rotation']
        self.rotation_list = []
        for quat in rotation_quat:
            self.rotation_list.append(2*np.arctan2(np.linalg.norm(quat[1:]), quat[0]))
        if self.enable_ove:
            self.ove_data_folder = './data_exp/optimal_viewpoint/'
            image_list, sf_list, bbox_list, depth_list = file_utils.get_files(self.ove_data_folder)
            self.im_paths, self.target_pois = file_utils.get_annos(self.ove_data_folder + 'anno/')

        self.initialize()

    def initialize(self):
        try:
            model_dict = torch.load(self.vis_mem_encoder_model)
            im_encoder_model_dict = {}
            vis_mem_encoder_model_dict = {}
            recovery_policy_model_dict = {}
            for k, v in model_dict.items():
                if 'encode_im' in k:
                    name = k.replace('encode_im.', '')
                    im_encoder_model_dict[name] = v
                if 'rel_path_weight_fc' in k or 'visual_memory_fc' in k:
                    vis_mem_encoder_model_dict[k] = v
                if 'GRU' in k:
                    recovery_policy_model_dict[k] = v
            self.encode_im.load_state_dict(im_encoder_model_dict)
            self.vis_mem_encoder.load_state_dict(vis_mem_encoder_model_dict)
            self.recovery_policy.load_state_dict(recovery_policy_model_dict)
        except:
            print("Cannot load pretrained encodeVisualMemory model")
            pass

        self.enable_recovery = True
        self.enable_ove = True

        return True           

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
            test_act = np.random.randint(0, self.action_dim)
            onehot_test_act = np.zeros(self.action_dim)
            onehot_test_act[test_act] = 1
            tensor_action = torch.tensor(onehot_test_act, dtype=torch.float32)
            tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
            if self.cuda:
                tensor_img = tensor_img.cuda()
                tensor_action = tensor_action.cuda()
            feat = self.encode_im(tensor_img)
            self.list2encode.append([feat, tensor_action])

        else:
            # flush when reaching new node
            if flush is True: # topometric_pose.dist < 0.1:
                self.list2encode = []
                #self.vis_mem = []

            elif exp_active is False:
                action = np.zeros(self.action_dim)
                action[guidance-1] = 1
                tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
                tensor_action = torch.tensor(action, dtype=torch.float32)
                if self.cuda:
                    tensor_action = tensor_action.cuda()
                    tensor_img = tensor_img.cuda()
                feat = self.encode_im(tensor_img)
                self.list2encode.append([feat, tensor_action])

        self.path_length = len(self.list2encode)

    def calcRecoveryGuidance(self, img=None):
        """
        Recovery Guide Provider Submodule:
        A module that guides a robot to return to the previous node, when StateDeterminant module
        determines that the robot is lost.
        If matching the retrieved image with the visual memory fails, call Exploration Guidance Module instead.
        Input:
        # - state: state from StateDeterminant Module
        - img: current image input (doesn't need if visual memory contains the current input image)
        Output:
        - action(s) guides to reach previous POI
        """

        if self.enable_exploration is False:
            self.enable_recovery = True

        if self.enable_recovery is True:
            # calculate the actions to return to the starting point of visual memory # doesn't need pose
            if img is None and self.path_length > 0:
                img = self.list2encode[-1][0]
            elif img is None:
                print('Nothing to encode or calculate because there was no input at all')
                raise Exception
            # encode the input image            
            tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
            if self.cuda:
                tensor_img = tensor_img.cuda()
            img_feature = self.encode_im(tensor_img)
            #_, img_feature = self.vis_mem_encoder(tensor_img, tensor_action)

            if len(self.vis_mem) == 0:
                positions = [np.array([0., 0.])]
                rotations = [np.array([0.])]

                for ts in range(self.path_length-1):
                    guid = self.list2encode[ts][1]
                    if guid[0] == 1:
                        pos = positions[ts] + 0.4 * np.concatenate([np.cos(rotations[ts]), np.sin(rotations[ts])])
                        rot = rotations[ts]
                    if guid[1] == 1:
                        pos = positions[ts]
                        rot = rotations[ts] + np.pi/6 * (-1. if self.path_direction == 'homing' else 1.)
                    if guid[2] == 1:
                        pos = positions[ts]
                        rot = rotations[ts] - np.pi/6 * (-1. if self.path_direction == 'homing' else 1.)
                    positions.append(pos)
                    rotations.append(rot)

                positions = torch.from_numpy(np.stack(positions))
                rotations = torch.from_numpy(np.stack(rotations))

                if self.cuda:
                    positions = positions.cuda().float()
                    rotations = rotations.cuda().float()

                rel_pos = torch.unsqueeze(positions, 0) - torch.unsqueeze(positions, 1)
                rel_pos_dist = torch.norm(rel_pos, dim=-1)  
                rel_pos_theta = torch.atan2(rel_pos[...,1], rel_pos[...,0]) - rotations.repeat(1,self.path_length)
                rel_pos_theta_cos = torch.cos(rel_pos_theta)
                rel_pos_theta_sin = torch.sin(rel_pos_theta)
                rel_pos_dt = torch.stack([rel_pos_dist, rel_pos_theta_cos, rel_pos_theta_sin], -1)

                rel_rot = rotations.permute(1, 0) - rotations + (np.pi if self.path_direction == 'homing' else 0.)         
                rel_rot_cos = torch.cos(rel_pos_theta)
                rel_rot_sin = torch.sin(rel_pos_theta)
                rel_orn = torch.stack([rel_rot_cos, rel_rot_sin], -1)

                self.rel_pose = torch.cat([rel_pos_dt, rel_orn], -1)

                for ts in range(self.path_length): 
                    vis_mem_seg, _ = self.vis_mem_encoder(self.list2encode[ts][1], [self.list2encode[t][0] for t in range(self.path_length)], self.rel_pose[ts,...])
                    self.vis_mem.append(vis_mem_seg)

            action = self.recovery_policy(self.vis_mem, img_feature)
            self.recovery_guidance = action
        """
        if info is False:
            self.enable_recovery, self.enable_exploration = False, True

        if done is True:
            self.enable_recovery, self.enable_exploration = True, False
        """

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
                actions, done, info = self.exploration_policy(img, torch.cat(self.vis_mem))
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
