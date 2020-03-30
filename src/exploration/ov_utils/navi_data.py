import numpy as np
import math

# MAP of SNU library street dataset
#                       stores
#  ------------------------------------------------
#  |(43,6)                                   (0,6)|
#  |                       N                      |         Right
#  |                     W - E                    |   Front       Rear
#  |                       S                      |         Left
#  |(43,0)                                 <-(0,0)|
#  ------------------------------------------------
#                      library

# Robot Head :   0     1     2     3     4     5     6     7     8     9    10    11
# Label      : '-05' '-06' '-07' '-08' '-09' '-10' '-11' '-00' '-01' '-02' '-03' '-04'

# Initial Setting : Robot-Head (0) & Position (0,0)

N_of_Col = 44
N_of_Row = 7

Interval_WE = 1.21
Interval_NS = 1.22


class Navi(object):
    def __init__(self):
        self.Robot_Head = 7
        self.Robot_Pos = np.array([3, 1])
        self.pos2theta = {'0': 317, '1': 287, '2': 257, '3': 227, '4': 197, '5': 167, '6': 137, '7': 107, '8': 77, '9': 47, '10': 17, '11': 347}

    def num2file(self, num_img):
        i = num_img // N_of_Row
        j = num_img % N_of_Row

        k = str(self.Robot_Head % 12).zfill(2)
        return str(i).zfill(2) + '-' + str(j), k

    def pos2file(self, pos_robot):
        i = pos_robot[0]
        j = pos_robot[1]

        k = str(self.Robot_Head % 12).zfill(2)
        return str(i).zfill(2) + '-' + str(j), k

    def file2curpos(self, file):
        self.Robot_Pos[0] = int(file.split("-")[0])
        self.Robot_Pos[1] = int(file.split("-")[1])
        self.Robot_Head = int(file.split("-")[2])

    def curpos2file(self):
        i = self.Robot_Pos[0]
        j = self.Robot_Pos[1]

        k = str(self.Robot_Head % 12).zfill(2)
        return str(i).zfill(2) + '-' + str(j) + '-' + k

    def go_straight(self, d, verbose=False):
        cur_theta = self.Robot_Head * 30
        delta_WE = round(d * math.cos(cur_theta * math.pi / 180) / Interval_WE)
        delta_NS = round(d * math.sin(cur_theta * math.pi / 180) / Interval_NS)

        self.Robot_Pos[0] += delta_WE
        if self.Robot_Pos[0] > N_of_Col - 1:
            self.Robot_Pos[0] = N_of_Col - 1
        if self.Robot_Pos[0] < 0:
            self.Robot_Pos[0] = 0

        self.Robot_Pos[1] += delta_NS
        if self.Robot_Pos[1] > N_of_Row - 1:
            self.Robot_Pos[1] = N_of_Row - 1
        if self.Robot_Pos[1] < 0:
            self.Robot_Pos[1] = 0
        if verbose:
            print(">> Go straight %.3fm" % (d))

    def turn(self, theta_, rotate_dir, verbose=False):
        theta = int(round(180 / np.pi * theta_))
        theta = (abs(theta), -abs(theta))[rotate_dir == "right"]
        self.Robot_Head -= round(theta / 30)
        self.Robot_Head %= 12
        if verbose:
            print(">> Rotate %d° on %s" % (int(round(180 / np.pi * theta_)), rotate_dir))
        return theta

    def turn_straight(self, d, theta_, rotate_dir, verbose=False):
        # if not self.rotated:
        # if self.POI_imgloc == "left":
        #     turn_theta = int(180 / np.pi * (theta_ - self.center_poi_theta))
        # else:
        #     turn_theta = int(180 / np.pi * (theta_ + self.center_poi_theta))
        # else:
        turn_theta = int(180 / np.pi * theta_)
        turn_theta = (abs(turn_theta), -abs(turn_theta))[rotate_dir == "right"]
        cur_theta = self.pos2theta[str(self.Robot_Head)]
        self.Robot_Head -= round(turn_theta / 30)
        self.Robot_Head %= 12

        theta = int(np.round(int(180 / np.pi * theta_) / 30) * 30)
        theta = (abs(theta), -abs(theta))[rotate_dir == "right"]
        target_theta = cur_theta + theta
        delta_WE = -np.round(d * math.cos(target_theta * math.pi / 180) / Interval_WE)
        delta_NS = np.floor(d * math.sin(target_theta * math.pi / 180) / Interval_NS)

        self.Robot_Pos[0] += delta_WE
        if self.Robot_Pos[0] > N_of_Col - 1:
            self.Robot_Pos[0] = N_of_Col - 1
        if self.Robot_Pos[0] < 0:
            self.Robot_Pos[0] = 0

        self.Robot_Pos[1] += delta_NS
        if self.Robot_Pos[1] > N_of_Row - 1:
            self.Robot_Pos[1] = N_of_Row - 1
        if self.Robot_Pos[1] < 0:
            self.Robot_Pos[1] = 0
        if verbose:
            print(">> Rotate %d° on %s and Go straight %.3fm" % (int(round(180 / np.pi * theta_)), rotate_dir, d))
        disp_x = -d * math.cos(target_theta * math.pi / 180) / Interval_WE
        disp_y = d * math.sin(target_theta * math.pi / 180) / Interval_NS
        return disp_x, disp_y

    def dist(self, p1, p2):
        p1_w, p1_h, _ = p1.split("-")
        p2_w, p2_h, _ = p2.split("-")
        h_dist = abs(int(p2_h) - int(p1_h)) * Interval_NS
        w_dist = abs(int(p2_w) - int(p1_w)) * Interval_WE
        return np.sqrt(h_dist ** 2 + w_dist ** 2)
