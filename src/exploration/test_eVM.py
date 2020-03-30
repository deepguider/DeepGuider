from active_navigation import ActiveNavigationModule
import argparse
import numpy as np
import os
import torch
from PIL import Image



anm = ActiveNavigationModule(None)


test_eVM = True
if test_eVM:
    img_list = np.sort([os.path.join('./data_exp/eVM_test', x) for x in os.listdir('./data_exp/eVM_test')])
    with torch.no_grad():
        for img_name in img_list:
            test_img = Image.open(img_name).convert("RGB")
            anm.encodeVisualMemory(test_img, None, None, test_mode=True)

    print("Test encodeVisualMemory Module Done")