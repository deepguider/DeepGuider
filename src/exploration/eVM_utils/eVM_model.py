import torch
import torch.nn as nn


class FCN_2layer(nn.Module):
    def __init__(self, input_ch, hidden_ch, output_ch, activation=None):
        super(FCN_2layer, self).__init__()
        self.layer1 = nn.Linear(input_ch, hidden_ch, bias=True)
        self.layer2 = nn.Linear(hidden_ch, output_ch, bias=True)
        self.relu = nn.ReLU(inplace=True)

        if activation == 'tanh':
            self.activation = nn.Tanh()
        elif activation == 'sigmoid':
            self.activation = nn.Sigmoid()
        elif activation == 'softmax':
            self.activation = nn.Softmax()
        elif activation == 'relu':
            self.activation = nn.ReLU()
        elif activation == None:
            self.activation = None

    def forward(self, x):
        x = self.layer1(x)
        x = self.relu(x)
        x = self.layer2(x)
        if self.activation == None:
            return x
        else:
            return self.activation(x)

class BasicConv(nn.Module):
    def __init__(self, in_planes, out_planes, kernel_size, stride=1, padding=0, dilation=1, groups=1, relu=True, bn=True, bias=False):
        super(BasicConv, self).__init__()
        BatchNorm = nn.BatchNorm2d
        self.out_channels = out_planes
        self.conv = nn.Conv2d(in_planes, out_planes, kernel_size=kernel_size, stride=stride, padding=padding, dilation=dilation, groups=groups, bias=bias)
        self.bn = BatchNorm(out_planes, eps=1e-5, momentum=0.01, affine=True) if bn else None
        self.relu = nn.ReLU(inplace=True) if relu else None

    def forward(self, x):
        x = self.conv(x)
        if self.bn is not None:
            x = self.bn(x)
        if self.relu is not None:
            x = self.relu(x)
        return x

class encodeVisualMemory(nn.Module):
    def __init__(self):
        super(encodeVisualMemory, self).__init__()

        self.img_size = 224
        self.out_size = 7 # int(self.img_size / 2 ** 5)
        self.action_dim = 3

        self.conv1 = BasicConv(3, 32, kernel_size=3, padding=1)
        self.conv2 = BasicConv(32, 64, kernel_size=3, padding=1)
        self.conv3 = BasicConv(64, 128, kernel_size=3, padding=1)
        self.conv4 = BasicConv(128, 256, kernel_size=3, padding=1)
        self.conv5 = BasicConv(256, 512, kernel_size=3, padding=1)

        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=2, stride=2)

        self.fc1 = nn.Linear(512 * self.out_size * self.out_size, 512 * 2, bias=True)
        self.fc2 = nn.Linear(512 * 2, 512, bias=True)

        self.visual_memory_fc = FCN_2layer((512 + self.action_dim), 512, 256, activation='relu')

    def forward(self, x, act):
        x = self.conv1(x)
        x = self.maxpool(x)
        x = self.conv2(x)
        x = self.maxpool(x)
        x = self.conv3(x)
        x = self.maxpool(x)
        x = self.conv4(x)
        x = self.maxpool(x)
        x = self.conv5(x)
        x = self.maxpool(x)

        x = x.view(x.size(0), -1)

        x = self.relu(self.fc1(x))
        x = self.fc2(x)

        mem = torch.cat([x, act], -1)
        mem = self.visual_memory_fc(mem)

        return mem