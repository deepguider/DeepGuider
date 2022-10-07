import torch.nn as nn
import torch
from .modules.resnet_network import ResNetDropblock
from torchvision.models.vgg import vgg16
from torchvision.models.resnet import resnet18, resnet50, resnet34
from torchvision.models.densenet import densenet121

def initialize_network(network_opt='resnet18', output_class=7):
    assert network_opt in ['resnet18', 'resnet34', 'resnet50', 'densenet121']
    if 'resnet' in network_opt:
        if network_opt == 'resnet18':
            resnet_type = 18
            base_net = 'resnet18'
        elif network_opt == 'resnet34':
            resnet_type = 34
            base_net = 'resnet34'
        else:  # elif network_opt == 'resnet50':
            resnet_type = 50
            base_net = 'resnet50'
        network = ResNetDropblock(output_class=output_class, resnet_type=resnet_type,
                                  drop_block=True, drop_pos=None, layer_depth=4,
                                  drop_prob=0.75, drop_block_size=3)
    
    else:  # elif network_opt == 'densenet121'
        network = densenet121(pretrained=False, num_classes=output_class)

    trained_params = network.parameters()

    return network, trained_params
