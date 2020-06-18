import torch
import torch.nn as nn
from .non_local_block.non_local_embedded_gaussian import NONLocalBlock2D
# from utils.tensorboardx_writer import tensorboardx_visualize_average_features
from torchvision.models.resnet import resnet34, resnet18, resnet50
from dropblock import DropBlock2D, LinearScheduler


class ResNetDropblock(nn.Module):
    # TNet with Non-Local-Block on the 4th stage (before the last conv of the 4th stage)
    def __init__(self, output_class=7, model_path=None, resnet_type=34, drop_block=False, drop_prob=0.5, drop_pos=None, layer_depth=4, drop_block_size=7):
        super(ResNetDropblock, self).__init__()

        assert resnet_type in [18, 34, 50]
        assert layer_depth in [1, 2, 3, 4]
        if resnet_type == 18:
            self.base = resnet18(pretrained=False, num_classes=1000)
            # state_dict = torch.load('resnet18.pth')
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
            # state_dict = torch.load('resnet34.pth')
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
            # state_dict = torch.load('resnet50.pth')
            if layer_depth == 4:
                last_fc_in_channel = 512 * 4
            elif layer_depth == 3:
                last_fc_in_channel = 256 * 4
            elif layer_depth == 2:
                last_fc_in_channel = 128 * 4
            else:  # elif layer_depth == 1:
                last_fc_in_channel = 64 * 4

        # self.base.load_state_dict(state_dict)

        # def weight_init(m):
        #     if isinstance(m, nn.Conv2d) or isinstance(m, nn.ConvTranspose2d):
        #         nn.init.xavier_uniform_(m.weight, gain=nn.init.calculate_gain('relu'))
        #         if m.bias is not None:
        #             nn.init.zeros_(m.bias)
        #
        # self.base.layer3.apply(weight_init)
        # self.base.layer4.apply(weight_init)

        self.base.fc = nn.Linear(in_features=last_fc_in_channel, out_features=output_class, bias=True)

        if drop_block:
            self.dropblock = LinearScheduler(
                DropBlock2D(drop_prob=drop_prob, block_size=drop_block_size),
                start_value=0.,
                stop_value=drop_prob,
                nr_steps=300
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

    def forward(self, x):
        if type(self.dropblock) != nn.Sequential:
            self.dropblock.step()

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
        x = torch.flatten(x, 1)
        out = self.base.fc(x)

        return out

