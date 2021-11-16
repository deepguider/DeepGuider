'''VGG11/13/16/19 in Pytorch.'''
import torch
import torch.nn as nn
import torchvision.models as models
import torch.nn.functional as F

from ipdb import set_trace as bp

cfg = {
    'VGG16_POST': [512, 512, 'M']
}

class VGG_POST(nn.Module):
    def __init__(self, height, width):
        super(VGG_POST, self).__init__()  # input  ==> encoder( vgg16 : h,w / 2^5 ) ==> h,w / 16
        self.feature_post = self._make_layers(cfg['VGG16_POST'])  #  h, w / 16 / 2  ==> h,w / 32
        self.feature_size = (int(height/32) * int(width/32))*512  # 
        self.avgpool2d = nn.AvgPool2d(kernel_size=1, stride=1)  # view() for FC must follow in forward()
        self.fc1 = nn.Linear(self.feature_size, 512)
        self.fc2 = nn.Linear(512, 128) 
        self.fc3 = nn.Linear(128, 32)
        self.fc4 = nn.Linear(32, 3)  # left, right, uncertain, 4*3

    def forward(self, x):  # x from pretrained VGG16,
        #  x : torch.Size([16, 512, , 16])
        x = self.feature_post(x)  # torch.Size([16, 512, 1, 1])
        x = self.avgpool2d(x)
        x = x.view(x.size(0), -1)  # torch.Size([16, 512])
        x = F.relu(self.fc1(x))  # 256
        x = F.relu(self.fc2(x))  # 128
        x = F.relu(self.fc3(x))  # 64
        x = F.relu(self.fc4(x))  # 3
        return x

    def _make_layers(self, cfg):
        layers = []
        in_channels = 512
        for x in cfg:
            if x == 'M':
                layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
            else:
                layers += [nn.Conv2d(in_channels, x, kernel_size=3, padding=1),
                           nn.BatchNorm2d(x),
                           nn.ReLU(inplace=True)]
                in_channels = x
        # layers += [nn.AvgPool2d(kernel_size=1, stride=1)]  # AvgPool2d is defined indivisual layer for using GradCam
        return nn.Sequential(*layers)
