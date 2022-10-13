import torch
import torch.nn as nn
# from utils.tensorboardx_writer import tensorboardx_visualize_average_features
from torchvision.models.resnet import resnet34, resnet18, resnet50
from dropblock import DropBlock2D, LinearScheduler


class ResNetDropblock(nn.Module):  # fusion after GAP
    def __init__(self, output_class=2, model_path=None, resnet_type=18, drop_block=False, drop_prob=0.5, drop_pos=None, layer_depth=4, drop_block_size=3, from_scratch=False, drop_step=500000):
        super(ResNetDropblock, self).__init__()

        assert resnet_type in [18, 34, 50]
        assert layer_depth in [1, 2, 3, 4]
        if resnet_type == 18:
            self.base = resnet18(pretrained=False, num_classes=1000)
            if not from_scratch:
                state_dict = torch.load('resnet18.pth')
            if layer_depth == 4:
                last_fc_in_channel = 512 * 1
            elif layer_depth == 3:
                last_fc_in_channel = 256 * 1
            elif layer_depth == 2:
                last_fc_in_channel = 128 * 1
            else:  # elif layer_depth == 1:
                last_fc_in_channel = 64 * 1

        elif resnet_type == 34:
            self.base = resnet34(pretrained=False, num_classes=1000)
            if not from_scratch:
                state_dict = torch.load('resnet34.pth')
            if layer_depth == 4:
                last_fc_in_channel = 512 * 1
            elif layer_depth == 3:
                last_fc_in_channel = 256 * 1
            elif layer_depth == 2:
                last_fc_in_channel = 128 * 1
            else:  # elif layer_depth == 1:
                last_fc_in_channel = 64 * 1
        else:  # elif resnet_type == 50:
            self.base = resnet50(pretrained=False, num_classes=1000)
            if not from_scratch:
                state_dict = torch.load('resnet50.pth')
            if layer_depth == 4:
                last_fc_in_channel = 512 * 4
            elif layer_depth == 3:
                last_fc_in_channel = 256 * 4
            elif layer_depth == 2:
                last_fc_in_channel = 128 * 4
            else:  # elif layer_depth == 1:
                last_fc_in_channel = 64 * 4

        if not from_scratch:
            self.base.load_state_dict(state_dict)

        # def weight_init(m):
        #     if isinstance(m, nn.Conv2d) or isinstance(m, nn.ConvTranspose2d):
        #         nn.init.xavier_uniform_(m.weight, gain=nn.init.calculate_gain('relu'))
        #         if m.bias is not None:
        #             nn.init.zeros_(m.bias)
        #
        # self.base.layer3.apply(weight_init)
        # self.base.layer4.apply(weight_init)

        self.base.fc = nn.Linear(in_features=last_fc_in_channel * 3, out_features=output_class, bias=True)

        if drop_block:
            self.dropblock = LinearScheduler(
                DropBlock2D(drop_prob=drop_prob, block_size=drop_block_size),
                start_value=0.,
                stop_value=drop_prob,
                nr_steps=drop_step
            )
        else:
            self.dropblock = nn.Sequential()

        self.drop_pos = drop_pos
        self.layer_depth = layer_depth

        if model_path is not None:
            self.model_path = model_path
            assert '.pth' in self.model_path or '.pkl' in self.model_path
            self.init_weight()

    def init_weight(self):
        print('Loading weights into state dict...')
        self.load_state_dict(torch.load(self.model_path, map_location=lambda storage, loc: storage))
        print('Finished!')

    def forward(self, x_left, x_center, x_right, return_feat=False):
        x_left = self.base.conv1(x_left)
        x_left = self.base.bn1(x_left)
        x_left = self.base.relu(x_left)
        x_left = self.base.maxpool(x_left)

        x_center = self.base.conv1(x_center)
        x_center = self.base.bn1(x_center)
        x_center = self.base.relu(x_center)
        x_center = self.base.maxpool(x_center)

        x_right = self.base.conv1(x_right)
        x_right = self.base.bn1(x_right)
        x_right = self.base.relu(x_right)
        x_right = self.base.maxpool(x_right)

        if self.drop_pos == 1 or self.drop_pos is None:
            x_left = self.dropblock(self.base.layer1(x_left))
            x_center = self.dropblock(self.base.layer1(x_center))
            x_right = self.dropblock(self.base.layer1(x_right))
        else:
            x_left = self.base.layer1(x_left)
            x_center = self.base.layer1(x_center)
            x_right = self.base.layer1(x_right)

        if self.layer_depth > 1:
            if self.drop_pos == 2 or self.drop_pos is None:
                x_left = self.dropblock(self.base.layer2(x_left))
                x_center = self.dropblock(self.base.layer2(x_center))
                x_right = self.dropblock(self.base.layer2(x_right))
            else:
                x_left = self.base.layer2(x_left)
                x_center = self.base.layer2(x_center)
                x_right = self.base.layer2(x_right)

            if self.layer_depth > 2:
                if self.drop_pos == 3 or self.drop_pos is None:
                    x_left = self.dropblock(self.base.layer3(x_left))
                    x_center = self.dropblock(self.base.layer3(x_center))
                    x_right = self.dropblock(self.base.layer3(x_right))
                else:
                    x_left = self.base.layer3(x_left)
                    x_center = self.base.layer3(x_center)
                    x_right = self.base.layer3(x_right)

                if self.layer_depth > 3:
                    if self.drop_pos == 4 or self.drop_pos is None:
                        x_left = self.dropblock(self.base.layer4(x_left))
                        x_center = self.dropblock(self.base.layer4(x_center))
                        x_right = self.dropblock(self.base.layer4(x_right))
                    else:
                        x_left = self.base.layer4(x_left)
                        x_center = self.base.layer4(x_center)
                        x_right = self.base.layer4(x_right)

        x_left = self.base.avgpool(x_left)
        x_center = self.base.avgpool(x_center)
        x_right = self.base.avgpool(x_right)
        feat_left = torch.flatten(x_left, 1)
        feat_center = torch.flatten(x_center, 1)
        feat_right = torch.flatten(x_right, 1)

        feat_combine = torch.cat([feat_left, feat_center, feat_right], 1)
        out = self.base.fc(feat_combine)

        if return_feat:
            return out, feat_combine
        else:
            return out


class ResNetDropblockOneInput(nn.Module):  # road non road classifier. fusion in the post processing
    def __init__(self, output_class=2, model_path=None, resnet_type=18, drop_block=False, drop_prob=0.5, drop_pos=None, layer_depth=4, drop_block_size=3, from_scratch=False, drop_step=500000):
        super(ResNetDropblockOneInput, self).__init__()

        assert resnet_type in [18, 34, 50]
        assert layer_depth in [1, 2, 3, 4]
        if resnet_type == 18:
            self.base = resnet18(pretrained=False, num_classes=1000)
            if not from_scratch:
                state_dict = torch.load('resnet18.pth')
            if layer_depth == 4:
                last_fc_in_channel = 512 * 1
            elif layer_depth == 3:
                last_fc_in_channel = 256 * 1
            elif layer_depth == 2:
                last_fc_in_channel = 128 * 1
            else:  # elif layer_depth == 1:
                last_fc_in_channel = 64 * 1

        elif resnet_type == 34:
            self.base = resnet34(pretrained=False, num_classes=1000)
            if not from_scratch:
                state_dict = torch.load('resnet34.pth')
            if layer_depth == 4:
                last_fc_in_channel = 512 * 1
            elif layer_depth == 3:
                last_fc_in_channel = 256 * 1
            elif layer_depth == 2:
                last_fc_in_channel = 128 * 1
            else:  # elif layer_depth == 1:
                last_fc_in_channel = 64 * 1
        else:  # elif resnet_type == 50:
            self.base = resnet50(pretrained=False, num_classes=1000)
            if not from_scratch:
                state_dict = torch.load('resnet50.pth')
            if layer_depth == 4:
                last_fc_in_channel = 512 * 4
            elif layer_depth == 3:
                last_fc_in_channel = 256 * 4
            elif layer_depth == 2:
                last_fc_in_channel = 128 * 4
            else:  # elif layer_depth == 1:
                last_fc_in_channel = 64 * 4

        if not from_scratch:
            self.base.load_state_dict(state_dict)

        self.base.fc = nn.Linear(in_features=last_fc_in_channel, out_features=output_class, bias=True)

        if drop_block:
            self.dropblock = LinearScheduler(
                DropBlock2D(drop_prob=drop_prob, block_size=drop_block_size),
                start_value=0.,
                stop_value=drop_prob,
                nr_steps=drop_step
            )
        else:
            self.dropblock = nn.Sequential()

        self.drop_pos = drop_pos
        self.layer_depth = layer_depth

        if model_path is not None:
            self.model_path = model_path
            assert '.pth' in self.model_path or '.pkl' in self.model_path
            self.init_weight()

    def init_weight(self):
        print('Loading weights into state dict...')
        self.load_state_dict(torch.load(self.model_path, map_location=lambda storage, loc: storage))
        print('Finished!')

    def forward(self, x, return_feat=False):

        x = self.base.conv1(x)
        x = self.base.bn1(x)
        x = self.base.relu(x)
        x = self.base.maxpool(x)


        if self.drop_pos == 1 or self.drop_pos is None:
            x = self.dropblock(self.base.layer1(x))
        else:
            x = self.base.layer1(x)

        if self.layer_depth > 1:
            if self.drop_pos == 2 or self.drop_pos is None:
                x = self.dropblock(self.base.layer2(x))
            else:
                x = self.base.layer2(x)

            if self.layer_depth > 2:
                if self.drop_pos == 3 or self.drop_pos is None:
                    x = self.dropblock(self.base.layer3(x))
                else:
                    x = self.base.layer3(x)

                if self.layer_depth > 3:
                    if self.drop_pos == 4 or self.drop_pos is None:
                        x = self.dropblock(self.base.layer4(x))
                    else:
                        x = self.base.layer4(x)

        x = self.base.avgpool(x)
        feat = torch.flatten(x, 1)

        out = self.base.fc(feat)

        if return_feat:
            return out, feat
        else:
            return out
