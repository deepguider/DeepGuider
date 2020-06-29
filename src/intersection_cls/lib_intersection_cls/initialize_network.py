import torch.nn as nn
import torch
from .modules.resnet_network import ResNetDropblock
from torchvision.models.vgg import vgg16
from torchvision.models.resnet import resnet18, resnet50, resnet34


def reinitialize_base_weight(network_base, network_type='vgg16'):
    """
    reinitialize layer lv3 and after. May be only use it when full train to make sure that things are fine-tuned
    :param network:
    :param network_type:
    :return:
    """
    assert network_type in ['vgg16', 'resnet18', 'resnet34', 'resnet50']
    if network_type == 'vgg16':
        # https://discuss.pytorch.org/t/how-to-re-set-alll-parameters-in-a-network/20819
        for a in range(10, len(network_base.features)):
            if isinstance(network_base.features[a], nn.Conv2d) or isinstance(network_base.features[a], nn.Linear):
                network_base.features[a].reset_parameters()
    else:  # elif network_type in ['resnet18', 'resnet34', 'resnet50']
        for idx, m in enumerate(network_base.modules()):
            if network_type == 'resnet18' and idx < 25:
                continue
            elif network_type == 'resnet34' and idx < 52:
                continue
            elif network_type == 'resnet50' and idx < 69:
                continue
            if isinstance(m, nn.Conv2d) or isinstance(m, nn.Linear) or isinstance(m, nn.BatchNorm2d):
                m.reset_parameters()
    return network_base


def initialize_network(network_opt='resnet18', output_class=7):
    assert network_opt in ['resnet18', 'resnet34', 'resnet50']
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

    trained_params = network.parameters()

    return network, trained_params
