import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms
import numpy as np
import cv2
import os
from ipdb import set_trace as bp  # set_trace is not implemented in torch.nn.DataParallel(), so turn it off when parallel is used.
import gradcam_custom as gradcam
from CustomImageDataset import CustomImageDataset
import matplotlib.pyplot as plt
import time

from vgg_post import VGG_POST

class lrpose_recognizer:  # pose recognition
    def __init__(self, which_gpu=0):
        self.device = 'cuda:{}'.format(which_gpu) if torch.cuda.is_available() else 'cpu'  #cuda:0
        self.use_cuda = True if torch.cuda.is_available() else False
        self.best_acc = 0  # best test accuracy
        self.start_epoch = 0  # start from epoch 0 or last checkpoint epoch
        self.epoch = 0
        self.classes = ('left', 'uncertain', 'right')
        self.fps = 0
        self.h, self.w = 0, 0
        self.cmd_printed = False
        #  # Resize image for speed up
        if False:  # Destory aspect ratio for enhancing performance
            self.resize_h = 80  # from 480
            self.resize_w = 60  # from 640
        else:  # Keep aspect ratio
            self.resize_h = 60  # from 480
            self.resize_w = 80  # from 640

    def initialize(self, resume=True):
        self.resume = resume
        self.chkp_pth = './data_lrpose/ckpt.pth.best.acc96p'

        # Custom dataset ( left side, right side, uncertain(such as crosswalk) )
        print('==> Preparing data..')

        if True:
            encoder = torchvision.models.vgg16(pretrained=True)
            encoder_dim = 512
            layers = list(encoder.features.children())[:-2]  # Use only feature part except classifier of vgg
            encoder = nn.Sequential(*layers)
            net = nn.Module()
            net.add_module('encoder', encoder)
            vgg_post = VGG_POST(self.resize_h, self.resize_w)
            net.add_module('post', vgg_post)

        self.net = net.to(self.device)

        if self.resume:
            # Load checkpoint.
            # print('==> Resuming from checkpoint..')
            checkpoint = torch.load(self.chkp_pth)
            self.net.load_state_dict(checkpoint['net'])
            self.best_acc = checkpoint['acc']
            self.start_epoch = checkpoint['epoch']
            self.epoch = self.start_epoch

        return 1  # Returning non-zero value means success

    def apply(self, img_cv2, ts=0.0):
        pred, conf = self._apply(img_cv2)
        return pred, np.max(conf)

    def _apply(self, img_cv2):  # for single test image
        self.h, self.w = img_cv2.shape[:2]
        start_t = time.time()
        img_tensor = self.preprocess_image(img_cv2)
        img = img_tensor.to(self.device)
        self.net.eval()
        with torch.no_grad():
            features = self.net.encoder(img)
            output = self.net.post(features)
            output = F.softmax(output, dim=1)

        _ , pred = output.max(1)   # pred = one of [0 , 1 , 2] which means ['left', 'uncertain', 'right']
        confidence = (output.cpu().numpy()*100)[0]  # example of confidence : [ 0%, 80%, 20%]
        self.fps = 1 / (time.time() - start_t)
        pred = int(pred)
        self.pred = pred
        self.conf = confidence
        return self.pred, self.conf  # 1 integer, 3 floats

    def get_result(self):
        return self.pred, self.conf  # 1 integer, 3 floats

    def apply_gradcam(self, img_cv2):  # for single test image
        pred, confidence = self._apply(img_cv2)
        img_tensor = self.preprocess_image(img_cv2)
        img = img_tensor.to(self.device)
        self.net.eval()

        # print(self.net)  # You can choose layer of interest from the print message
        model = self.net
        grad_cam1 = gradcam.GradCam(model=model, feature_module=model.encoder, target_layer_names=['28'],  use_cuda=self.use_cuda )
        model = self.net.post
        grad_cam2 = gradcam.GradCam(model=model, feature_module=model.feature_post, target_layer_names=['3'],  use_cuda=self.use_cuda )

        label_index = None
        features = self.net.encoder(img)
        # mask = grad_cam1(img, label_index)  # When target_index is None, grad_cam will choose argmax
        mask = grad_cam2(features, label_index)  # When target_index is None, grad_cam will choose argmax

        img_float = np.float32(img_cv2 / 255)
        img_jet = self.show_cam_on_image(img_float, mask)

        return img_jet, pred, confidence

    def preprocess_image(self, img_cv2):  # transform for input image (img) from cv2.imread(), uint8
        img = np.float32(cv2.resize(img_cv2, (self.resize_w, self.resize_h))) / 255
        # means = [0.485, 0.456, 0.406]
        # stds = [0.229, 0.224, 0.225]
        means = [0.4914, 0.4822, 0.4465]
        stds = [0.2023, 0.1994, 0.2010]

        preprocessed_img = img.copy()[:, :, ::-1]
        for i in range(3):
            preprocessed_img[:, :, i] = preprocessed_img[:, :, i] - means[i]
            preprocessed_img[:, :, i] = preprocessed_img[:, :, i] / stds[i]
        preprocessed_img = np.ascontiguousarray(np.transpose(preprocessed_img, (2, 0, 1)))
        preprocessed_img = torch.from_numpy(preprocessed_img)
        preprocessed_img.unsqueeze_(0)
        img_tensor = preprocessed_img.requires_grad_(True)
        return img_tensor

    def show_cam_on_image(self, img_float, mask):
        imshape = img_float.shape
        h = imshape[0]
        w = imshape[1]
        mask = cv2.resize(mask, (w, h))

        heatmap = cv2.applyColorMap(np.uint8(255 * mask), cv2.COLORMAP_JET)
        heatmap = np.float32(heatmap) / 255
        cam = heatmap + np.float32(img_float)
        cam = cam / np.max(cam)
        img_jet = np.uint8(255 * cam)
        cv2.imwrite("cam.jpg", img_jet)
        return img_jet

    def read_testdata(self, data_set_path='/home/ccsmm/workdir/DB_Repo/ETRI_CartRobot/extracted/200626'):
        cid = CustomImageDataset(data_set_path)
        fnlist = cid.image_files_path
        return fnlist

    def cv2_print_cmd(self):
        if self.cmd_printed == False:
            print("\nCommand on cv2 window")
            print(" q : (ESC) exit")
            print(" f : + 100 frames")
            print(" b : - 100 frames")
            print(" p : (Space bar) pause")
            print(" g : Toggle GradCam function\n")
            self.cmd_printed = True

    def keypress(self, idx, gradcam_en):
        ## Keyboard
        self.cv2_print_cmd()
        isBreak = False
        key = cv2.waitKey(1) & 0xff
        if key == ord('q') or key == 27:  # 'q' or ESC to quit loop
            cv2.destroyAllWindows()
            isBreak = True
        elif key == ord('f'):  #  >> +1000 frame skip
            idx += 100
        elif key == ord('b'):  #  >> -1000 frame skip
            idx = max(idx-100, 0)
        elif key == ord('g'):  # Toggle gradcom_en signal
            gradcam_en = not gradcam_en
        elif key == ord('p') or key == 32:  # 'p' or Spacebar to puase, and any key to release
            if cv2.waitKey(1000000):
                isBreak = False
        return idx, gradcam_en, isBreak

    def get_classes(self):
        return self.classes


    def putText(self, img, text='*', org=(10,10), font=None, fontScale=1, color=(255,0,0), thickness=1):
        # org = (w, h)
        font = cv2.FONT_HERSHEY_SIMPLEX
        # fontScale = 1
        # color = (0, 0, 255)  # BGR
        # thickness = 1  # pixel
        img = cv2.putText(img, text, org, font, fontScale, color, thickness, cv2.LINE_AA, False)
        return img

    def cv2_imshow(self, img_cv2):
        h, w = img_cv2.shape[:2]
        classes = self.get_classes()
        fps = self.get_fps()
        pred, conf = self.get_result()

        text = "{0:.1f} fps.".format(self.get_fps())
        img = self.putText(img_cv2, text, (w-200, h-20), color=(0, 0, 200), thickness=1)

        text = "{0} [{1:.1f}%]".format(classes[pred].upper(), conf[pred])
        img = self.putText(img, text, (int(w/4), 50), color=(0, 255, 0), thickness=2)

        cv2.imshow("Pose_Recog", img)
        self.draw_graph()

        #cv2.waitKey(1)  # replaced by self.keypress()

    def draw_graph(self):
        x_resol = 1
        x = range(len(self.conf))
        y = np.zeros(len(x))
        y[self.pred] = 100 
    
        alpha = 0.5 
        bar_width = 0.7*x_resol

        p1 = plt.bar(x, y, 
        bar_width, 
        color='b', 
        alpha=alpha,
        label='Pred')

        plt.pause(1e-5)

    def get_fps(self):
        return self.fps


if __name__ == '__main__':
    ## Initialize pose_recog class
    mod_pose_recog = lrpose_recognizer()
    mod_pose_recog.initialize()

    ## Prepare test data
    fnlist = mod_pose_recog.read_testdata('/home/ccsmm/workdir/DB_Repo/ETRI_CartRobot/extracted/200626')

    gradcam_en = False

    idx_max = len(fnlist)
    idx = 0

    #mod_pose_recog.cv2_print_cmd()
    fig = plt.figure()

    while ( idx < idx_max):
        idx += 1
        fig.clf()  # clear figure
        fn = fnlist[idx]  # '/home/ccsmm/workdir/DB_Repo/ETRI_CartRobot/extracted/200204/000000.jpg'
        img =  cv2.imread(fn, 1)

        #h, w = img.shape[:2]

        ## Do apply()
        start_t = time.time()
        if gradcam_en:  # Enable GradCam function
            img_jet, pred, conf = mod_pose_recog.apply_gradcam(img)
            img = img_jet
        else:  # normal
            pred, conf = mod_pose_recog.apply(img)

        ## Display
        print('prediction : {} , confidence : {}          \r'.format(pred, conf), end='')
        mod_pose_recog.cv2_imshow(img)
        idx, gradcam_en, isBreak = mod_pose_recog.keypress( idx, gradcam_en)  # waitKey()
        if isBreak:
            break
