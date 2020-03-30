#!/usr/bin/env python
import numpy as np 
# import tensorflow as tf
from src.exploration.ov_utils.myutils import make_mask, template_matching, get_surfacenormal, get_bbox, get_depth, get_img
from src.exploration.ov_utils.config import normal_vector
import src.exploration.ov_utils.file_utils as file_utils
import src.exploration.eVM_utils.utils as eVM_utils
from src.exploration.eVM_utils.eVM_model import encodeVisualMemory
from src.exploration.recovery_policy import Recovery
import torch
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

# import MapManager

"""
Active Navigation Module:
A module that maintains visual memory and provides three kinds of active navgitation guidance;
(1) recovery guidance, (2) exploration guidance, (3) optimal viewpoint guidance.
Initiation and termination conditions of (1) and (2) are checked by state from StateDeterminant module.

Author
------
Yunho Choi, Obin Kwon, Nuri Kim, Hwiyeon Yoo
"""


class ActiveNavigationModule():
    """Active Navigation Module"""
    def __init__(self, args, NV=None, map_manager=None):
        # map_manager = MapManager()
        # self.map = map_manager.getMap()
        self.args = args
        self.list2encode = []
        self.vis_mem = None
        # self.vis_mem_encoder = encodeVisualMemory()
        self.vis_mem_encoder_model = None
        # try:
        #     self.vis_mem_encoder.load_state_dict(torch.load(self.vis_mem_encoder_model))
        # except:
        #     print("Cannot load pretrained encodeVisualMemory model")
        #     pass

        self.enable_recovery = False
        # self.recovery_policy = Recovery()
        self.recovery_guidance = None
        
        self.enable_exploration = False
        self.exploration_policy = None  
        self.exploration_guidance = None

        try:
            self.enable_ove = args.enable_ove
        except:
            self.enable_ove = None
        self.NV = NV
        
    def encodeVisualMemory(self, img, guidance, topometric_pose, test_mode=False):
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

        if test_mode:
            test_act = np.random.randint(0, 3)
            onehot_test_act = np.zeros(3)
            onehot_test_act[test_act] = 1
            tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
            tensor_action = torch.tensor(onehot_test_act, dtype=torch.float32).unsqueeze(0)
            vis_mem = self.vis_mem_encoder(tensor_img, tensor_action)
            self.list2encode.append(vis_mem)
            self.vis_mem = vis_mem

        else:
            # flush when reaching new node
            if (self.map.getNode(topometric_pose.node_id).edges[topometric_pose.edge_idx].length - topometric_pose.dist) < 0.1: # topometric_pose.dist < 0.1:
                self.list2encode = []

            if self.enable_recovery is False and self.enable_exploration is False and self.enable_ove is False:
                action = guidance[-1]
                # self.list2encode.append([img,action])
                try:
                    tensor_img = eVM_utils.img_transform(img).unsqueeze(0)
                    tensor_action = torch.tensor(action).unsqueeze(0)
                    vis_mem = self.vis_mem_encoder(tensor_img, tensor_action)
                    self.list2encode.append(vis_mem)
                except:
                    print("NotImplementedError")
                    vis_mem = None

                self.vis_mem = vis_mem


    def calcRecoveryGuidance(self, state, img=None):
        """
        Recovery Guide Provider Submodule:
        A module that guides a robot to return to the previous node, when StateDeterminant module
        determines that the robot is lost.
        If matching the retrieved image with the visual memory fails, call Exploration Guidance Module instead.

        Input:
        - state: state from StateDeterminant Module
        - img: curreunt image input (doesn't need if visual memory contains the current input image)
        Output:
        - action(s) guides to reach previous POI
        """

        if state == 'lost' and self.enable_exploration is False:
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
                img_feature = self.vis_mem_encoder(tensor_img, tensor_action)
                # done: is back home, info: whether visual memory matching succeeded or not
                actions, done, info = self.recovery_policy(self.vis_mem, img_feature)
            except:
                raise
                # TODO: if recovery_policy fails to calculate the recovery actions,
                #       just reverse the actions in the visual memory (using self.list2encode)
                #       - can't implement now due to the ambiguity of the action space
                # print("NotImplementedError")
                actions, done, info = ['backward']*3, False, False

            self.recovery_guidance = actions

        if info is False:
            self.enable_recovery, self.enable_exploration = False, True

        if done is True and state == 'normal':
            self.enable_recovery, self.enable_exploration = False, False


    def calcExplorationGuidance(self, state, img):
        """
        Exploration Guidance Provider Submodule:
        A module that guides the robot to reach nearby POI using nearby visual information and visual memory.
        This module is triggered when the POI Recovery Guidance Module judges that it is hard to 
        return to a node associated with the previous POI, based on the visual memory matching result.

        Input:
        - state: state from StateDeterminant Module
        ---(topometric_pose_conf: confidence of topometric pose)
        - (tentative) POI detection result
        - img: curreunt image input
        - visual memory

        Output:
        - action(s) guides to reach nearby POI

        Dependency:
        - Localizer module
        """
        if state == 'normal':
            self.enable_exploration = False

        if self.enable_exploration is True:
            try:
                actions = self.exploration_policy(img, self.vis_mem)
            except:
                print("NotImplementedError")
                actions = ['f','f','f','f','f']

            self.exploration_guidance = actions
        else:
            self.exploration_guidance = None
        # return actions

    def calcNeedForOptimalViewpointGuidance(self, topometric_pose, poi_conf, entrance, entrance_conf, poi_successes):
        """
        A module calculates the need for optimal viewpoint guidance.
        It considers the results of POI detection and current location (destination, door).

        Input:
        - topometric_pose: node_id, edge-idx, dist
        - poi_conf: detection results of POI (confidence score)
        - (tentative) entrance: If there is an building entrance near current location, True, else False (Boolean) , tentative due to the API ambiguity.
        - entrance_conf: detection results of building entrances 
        - poi_successes: A sequence of POI detection result (confidence score)

        Output:
        - require the optimal viewpoint guidance optimization (Boolean)
        """

        # Cases
        case1 = np.mean(poi_successes) < 0.5
        case2 = ((self.map.getNode(topometric_pose.node_id).edges[topometric_pose.edge_idx].length - topometric_pose.dist) < 0.2) and (poi_conf < 0.5)
        case3 = entrance and (entrance_conf < 0.5)

        if case1 or case2 or case3:
            self.enable_ove = True
        else:
            self.enable_ove = False

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

        self.central_viewpoint_guidance = [0, 0, 0]
        self.optimal_viewpoint_guidance = [0, 0, 0]
        self.central_viewpoint = None
        self.optimal_viewpoint = None
        if self.enable_ove:
            # try:
            if self.args.central_guidance:
                c_heading, central_view = self.viewpoint_to_central(img_path, target_poi)
                if c_heading != None:
                    self.central_viewpoint_guidance = [0, 0, c_heading]
                    self.central_viewpoint = central_view
                    if self.args.optimal_guidance:
                        disp_x, disp_y, heading, optimal_view = self.viewpoint_optimizer(target_poi)
                        self.optimal_viewpoint_guidance = [disp_x, disp_y, heading]
                        self.optimal_viewpoint = optimal_view
            # except:
            #     pass

    def viewpoint_to_central(self, file_name, target_poi):
        heading = None
        templates, main_template, _ = file_utils.get_templates(self.args.data_folder, targetPOI=target_poi)
        img = get_img(self.args, file_name)
        sf = get_surfacenormal(self.args, file_name)
        bbox = get_bbox(self.args, file_name)
        h, w, _ = img.shape
        if len(bbox) > 0:
            # Read all boxes from the detection results
            bbs = []
            for bb in bbox:
                bbs.append(bb.rstrip().split(','))
            bbox = np.stack([np.float32(bbs[i]) for i in range(len(bbs))])
            bbox = np.reshape(bbox, [-1, 4, 2])

            # Template matching (Target POI and the boxes)
            template_matched, bbox, _, old_score = template_matching(img, bbox, templates, main_template)
            bbox = np.reshape(bbox, [-1, 4, 2])

            # Check that there is a bounding box which is matched with template (target POI)
            if len(bbox) > 0 and template_matched:
                bbox = bbox.astype(np.int32)
                # Take surface normal to decide the amount of rotation
                mask = make_mask(bbox, shape=[h, w])
                sf_abs = sf[mask == 1]
                sf_norm = np.mean(sf_abs, 0)
                sf_norm = sf_norm * 2 - 1
                sf_norm = sf_norm / np.linalg.norm(sf_norm, 2)

                self.NV.POI_imgloc = ("left", "right")[(bbox[0, 2, 0] + bbox[0, 0, 0]) / 2 > w / 2]
                # TODO: Deal with the various situations (there may exist sky in the center of the image)
                # Rotate the agent until the POI is locate on the center
                center_sf_norm = np.mean(sf[110:140, 235:265], (0, 1))
                center_sf_norm = center_sf_norm * 2 - 1
                center_sf_norm = center_sf_norm / np.linalg.norm(center_sf_norm, 2)

                # Check that the bounding box is on the left or right buildings (not street or sky)
                if abs(sf_norm[1]) < 0.8:
                    self.NV.center_poi_theta = np.arccos(np.dot(sf_norm, center_sf_norm))

                    # Decide the POI is on the left or the right
                    self.NV.POI_surf = ("left", "right")[sf_norm[0] < 0]

                    # Align the POI and the camera center.
                    self.NV.rotated = False
                    if round((180 / np.pi * self.NV.center_poi_theta) / 30) > 0 or ((bbox[0, 2, 0] + bbox[0, 0, 0]) / 2 - w / 2) > w / 4:
                        heading = self.NV.turn(30 / 180 * np.pi, self.NV.POI_imgloc, verbose=self.args.verbose)
                        self.NV.rotated = True
                    else:
                        heading = 0
        central_view = self.NV.curpos2file()
        return heading, central_view

    def viewpoint_optimizer(self, target_poi):
        optim_view = None
        disp_x = disp_y = heading = 0
        templates, main_template, opt_ratio = file_utils.get_templates(self.args.data_folder, targetPOI=target_poi)
        file_path = self.NV.curpos2file()
        img = get_img(self.args, file_path)
        depth_ = get_depth(self.args, file_path)
        bbox = get_bbox(self.args, file_path)
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

                # TODO: Estimate the exact distance
                D = np.mean(depth) * 19.2
                # Decide the amount of the movement using depth
                ratio = (abs(bbox[0, 3, 1] - bbox[0, 0, 1]) + abs(bbox[0, 1, 1] - bbox[0, 2, 1])) / 2 / h

                D0 = D * (1 - np.maximum(ratio / opt_ratio, 0.95))

                # Decide the moving direction
                sf = get_surfacenormal(self.args, file_path)
                sf_norm = np.mean(sf[mask == 1], 0)
                sf_norm = sf_norm * 2 - 1
                sf_norm = sf_norm / np.linalg.norm(sf_norm, 2)

                theta = np.arccos(np.dot(sf_norm, normal_vector))
                thetad = np.arctan(((D - D0) * np.sin(theta)) / (D - (D - D0) * np.cos(theta)))
                if thetad == 0:
                    D1 = D0
                else:
                    D1 = (D - D0) * np.sin(theta) / (np.sin(thetad))
                thetadd = theta + thetad

                ROT = ("left", "right")[self.NV.POI_surf == "left"]
                # Turn and go straight
                disp_x, disp_y = self.NV.turn_straight(D1, thetad, ROT, verbose=self.args.verbose)

                # Rotate to see the POI
                heading = self.NV.turn(thetadd, self.NV.POI_surf, verbose=self.args.verbose)
                optim_view = self.NV.curpos2file()
        return disp_x, disp_y, heading, optim_view

    def getVisualMemory(self):
        return self.vis_mem

    def isRecoveryGuidanceEnabled(self):
        return self.enable_recovery

    def getRecoveryGuidance(self):
        if self.enable_recovery:
            return self.recovery_guidance
        else:
            return None

    def isExplorationGuidanceEnabled(self):
        return self.enable_exploration

    def getExplorationGuidance(self):
        if self.enable_exploration:
            return self.exploration_guidance
        else:
            return None

    def isOptimalViewpointGuidanceEnabled(self):
        return self.enable_ove

    def getCentralViewpointGuidance(self):
        if self.enable_ove:
            return self.central_viewpoint_guidance
        else:
            return None

    def getOptimalViewpointGuidance(self):
        if self.enable_ove:
            return self.optimal_viewpoint_guidance    
        else:
            return None

    def getOptimalViewpointPath(self):
        if self.enable_ove:
            return self.optimal_viewpoint
        else:
            return None

    def getCentralViewpointPath(self):
        if self.enable_ove:
            return self.central_viewpoint
        else:
            return None

# if __name__ == "__main__":
#   anm = ActiveNavigationModule()


