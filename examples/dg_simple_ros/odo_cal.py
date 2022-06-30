import numpy as np
import sys
from ipdb import set_trace as bp

def cal_odo(mea_x=10.0, mea_y=0.0, gt_x=10.0, gt_y=0.0):
    '''
        mea_x, mea_y are measured x and y by encoder in meter
        gt_x, gt_y are real x and y measured by ruler in meter
    '''
    WD = 0.588  # Baseline between L and R wheel
    ep = 1e-9
    r = (mea_x*mea_x + mea_y*mea_y) / (2*np.abs(mea_y)+ep)
    theta = np.arcsin(np.abs(mea_x)/r)
    
    if mea_y < 0:
        odo_dL = theta * (r + WD/2)
        odo_dR = theta * (r - WD/2)
    else: 
        odo_dL = theta * (r - WD/2)
        odo_dR = theta * (r + WD/2)
    
    left_scale_factor = gt_x/odo_dL
    right_scale_factor = gt_x/odo_dR

    print("left_scale_factor, right_scale_factor : {} {} when mea_x, mea_y, gt_x are {} {} {}".format(left_scale_factor, right_scale_factor, mea_x, mea_y, gt_x))
    return left_scale_factor, right_scale_factor

def str2num(a):
    if a[0] == "+":
        ret = float(a[1:])
    elif a[0] == "-":
        ret = -float(a[1:])
    else:
        ret = float(a)
    return ret

if __name__ == "__main__":
    argv = sys.argv
    if len(argv) > 3:
        mea_x = str2num(argv[1])
        mea_y = str2num(argv[2])
        gt_x  = str2num(argv[3])
    else:
        print(">> Usage : {} [mea_x] [mea_y] [gt_x]\n   For example,".format(argv[0]))
        mea_x = 10.23
        mea_y = -0.25
        gt_x = 10.0

    l, r = cal_odo(mea_x, mea_y, gt_x)
