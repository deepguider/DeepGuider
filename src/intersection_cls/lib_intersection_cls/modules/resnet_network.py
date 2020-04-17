import torch
import torch.nn as nn
from .non_local_block.non_local_embedded_gaussian import NONLocalBlock2D
# from utils.tensorboardx_writer import tensorboardx_visualize_average_features
from torchvision.models.resnet import resnet34, resnet18, resnet50


class TNetNLBv2(nn.Module):
    # TNet with Non-Local-Block on the 4th stage (before the last conv of the 4th stage)
    def __init__(self, output_class=7, model_path=None, resnet_type=34):
        super(TNetNLBv2, self).__init__()

        assert resnet_type in [18, 34, 50]
        if resnet_type == 18:
            self.base = resnet18(pretrained=False, num_classes=1000)
            # state_dict = torch.load('resnet18.pth')
            last_fc_in_channel = 512 * 1
        elif resnet_type == 34:
            self.base = resnet34(pretrained=False, num_classes=1000)
            # state_dict = torch.load('resnet34.pth')
            last_fc_in_channel = 512 * 1
        else:  # elif resnet_type == 50:
            self.base = resnet50(pretrained=False, num_classes=1000)
            # state_dict = torch.load('resnet50.pth')
            last_fc_in_channel = 512 * 4
        # self.base.load_state_dict(state_dict)

        self.base.fc = nn.Linear(in_features=last_fc_in_channel, out_features=output_class, bias=True)

        self.nlb = NONLocalBlock2D(256, 128)

        if model_path is not None:
            self.model_path = model_path
            assert '.pth' in self.model_path or '.pkl' in self.model_path
            self.init_weight()

    def init_weight(self):
        print('Loading weights into state dict...')
        self.load_state_dict(torch.load(self.model_path, map_location=lambda storage, loc: storage))
        print('Finished!')

    def forward(self, x, tensorboardx_writer=None, iter=0, every_n_iter=500):
        x = self.base.conv1(x)
        x = self.base.bn1(x)
        x = self.base.relu(x)
        x = self.base.maxpool(x)

        x = self.base.layer1(x)
        x = self.base.layer2(x)
        x = self.base.layer3[:-1](x)
        x, nlb_y = self.nlb(x, return_nl_map=True)

        # if tensorboardx_writer is not None:
        #     # tensorboardx_visualize_average_features(x[:min(16, len(x))], tensorboardx_writer, "nlb", iter, every_n_iter)
        #     # tensorboardx_visualize_average_features(nlb_y[:min(16, len(x))], tensorboardx_writer, "nlb_y", iter, every_n_iter)
        #     tensorboardx_visualize_average_features(x, tensorboardx_writer, "nlb", iter, every_n_iter)
        #     tensorboardx_visualize_average_features(nlb_y, tensorboardx_writer, "nlb_y", iter, every_n_iter)

        x = self.base.layer3[-1](x)
        x = self.base.layer4(x)

        x = self.base.avgpool(x)
        x = torch.flatten(x, 1)
        out = self.base.fc(x)

        return out

