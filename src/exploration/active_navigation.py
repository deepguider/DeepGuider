#!/usr/bin/env python
import numpy as np 
import tensorflow as tf 
import matplotlib.pyplot as plt 
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
Yunho Choi, Obin Kwon, Nuri Kim
"""
class ActiveNavigationModule():
    """Active Navigation Module"""
    def __init__(self, map_manager):
        # map_manager = MapManager()
        self.map = map_manager.getMap()

        self.list2encode = []
        self.vis_mem = None
        self.vis_mem_encoder = None

        self.enable_recovery = False
        self.recovery_policy = None
        self.recovery_guidance = None
        
        self.enable_exploration = False
        self.exploration_policy = None  
        self.exploration_guidance = None

        self.enable_ove = None
        self.optimal_viewpoint_guidance = None
        self.viewpoint_optimizer = None
        self.get_viewpoint_scores = None
        
    def encodeVisualMemory(self, img, guidance, topometric_pose):
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

        # flush when reaching new node
        if (self.map.getNode(topometric_pose.node_id).edges[topometric_pose.edge_idx].length - topometric_pose.dist) < 0.1 # topometric_pose.dist < 0.1:
            self.list2encode = []

        if enable_recovery is False and enable_exploration is False and enable_ove is False:
            action = guidance[-1]
            self.list2encode.append([img,action])

            try:
                vis_mem = self.vis_mem_encoder(self.list2encode)
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
        ---(topometric_pose_conf: confidence of topometric pose)
        - img: curreunt image input (doesn't need if visual memory contains the current input image)
        Output:
        - action(s) guides to reach previous POI
        """

        if state == 'lost' and self.enable_exploration is False:
            self.enable_recovery = True

        if self.enable_recovery is True:
            try:
                # calculate the actions to return to the starting point of visual memory # doesn't need pose
                if img == None:
                    img = self.list2encode[-1][0]
                # done: is back home, info: whether visual memory matching succeeded or not
                actions, done, info = self.recovery_policy(self.vis_mem, img) 
            except:
                # TODO: if recovery_policy fails to calculate the recovery actions,
                #       just reverse the actions in the visual memory (using self.list2encode)
                #       - can't implement now due to the ambiguity of the action space
                print("NotImplementedError")
                actions, done, info = ['b','b','b','b','b'], False, False

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

    def calcOptimalViewpointGuidance(self, img):
        """
        Optimal Viewpoint Guidance Provider Submodule:
        A module that provides an optimal viewpoint guidance for enhancing POI detection.

        Input:
        - img: curreunt image input

        Output:
        - Action(s) guides to find an optimal viewpoint 
        """

        if self.enable_ove:
            try:
                viewpoint_scores = self.get_viewpoint_scores(img)
                displacement_x, displacement_y, heading = self.viewpoint_optimizer(viewpoints_scores)
                self.optimal_viewpoint_guidance = [displacement_x, displacement_y, heading]
            except:
                print("NotImplementedError") 
                self.optimal_viewpoint_guidance  = [0,0,0]  # If possible, heading of the robot
        else:
            self.optimal_viewpoint_guidance  = None

        # return self.enable_ove, action


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

    def getOptimalViewpointGuidance(self):
        if self.enable_ove:
            return self.optimal_viewpoint_guidance    
        else:
            return None

# if __name__ == "__main__":
#   anm = ActiveNavigationModule()


