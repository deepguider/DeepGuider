import torch
from torch import nn as nn
from torch import Size, Tensor
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import abc
from gym import spaces

import cv2

import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
tf.contrib.resampler  # import C++ op
import numpy as np
from scipy.spatial.distance import cdist
import matplotlib
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import time

def match_descriptors_gpu(descriptors1, descriptors2, p=2,
                      max_distance=np.inf, cross_check=True, max_ratio=1.0):
    if descriptors1.shape[1] != descriptors2.shape[1]:
        raise ValueError("Descriptor length must equal.")

    distances = torch.cdist(descriptors1, descriptors2, p=p)

    indices1 = torch.arange(descriptors1.shape[0]).to(descriptors1.device)
    indices2 = torch.argmin(distances, dim=1)

    if cross_check:
        matches1 = torch.argmin(distances, dim=0)
        mask = indices1 == matches1[indices2]
        indices1 = indices1[mask]
        indices2 = indices2[mask]

    if max_distance < np.inf:
        mask = distances[indices1, indices2] < max_distance
        indices1 = indices1[mask]
        indices2 = indices2[mask]

    if max_ratio < 1.0:
        best_distances = distances[indices1, indices2]
        distances[indices1, indices2] = torch.tensor([float('inf')]).to(descriptors1.device)
        second_best_indices2 = torch.argmin(distances[indices1], dim=1)
        second_best_distances = distances[indices1, second_best_indices2]
        second_best_distances[second_best_distances == 0] \
            = torch.finfo(torch.float32).eps
        ratio = best_distances / second_best_distances
        mask = ratio < max_ratio
        indices1 = indices1[mask]
        indices2 = indices2[mask]

#     matches = np.column_stack((indices1, indices2))
    matches = torch.stack([indices1, indices2], dim=1)
    best_distances = best_distances[mask] #indices1, indices2]
    # best_distances /= len(indices1)

    return matches, indices1, indices2, -best_distances#, np.linalg.norm(best_distances)

class HFNet:
    def __init__(self, model_path, outputs):
        config = tf.ConfigProto(allow_soft_placement=True)
        config.gpu_options.allow_growth = True
        self.session = tf.Session(config=config)
        self.image_ph = tf.placeholder(tf.float32, shape=(None, None, 3))

        net_input = tf.image.rgb_to_grayscale(self.image_ph[None])
        tf.saved_model.loader.load(
            self.session, [tag_constants.SERVING], str(model_path),
            clear_devices=True,
            input_map={'image:0': net_input})

        graph = tf.get_default_graph()
        self.outputs = {n: graph.get_tensor_by_name(n+':0')[0] for n in outputs}
        self.nms_radius_op = graph.get_tensor_by_name('pred/simple_nms/radius:0')
        self.num_keypoints_op = graph.get_tensor_by_name('pred/top_k_keypoints/k:0')
#         print('\n !!!!!! num key points!!!!!!: ',graph.get_tensor_by_name('pred/top_k_keypoints/k:0'), self.session.run(graph.get_tensor_by_name('pred/top_k_keypoints/k:0')))
        
    def inference(self, image, nms_radius=4, num_keypoints=32):
        inputs = {
            self.image_ph: image[..., ::-1].astype(np.float),
            self.nms_radius_op: nms_radius,
            self.num_keypoints_op: num_keypoints,
        }
        return self.session.run(self.outputs, feed_dict=inputs)

class CustomFixedCategorical(torch.distributions.Categorical):  # type: ignore
    def sample(
        self, sample_shape: Size = torch.Size()  # noqa: B008
    ) -> Tensor:
        return super().sample(sample_shape).unsqueeze(-1)

    def log_probs(self, actions: Tensor) -> Tensor:
        return (
            super()
            .log_prob(actions.squeeze(-1))
            .view(actions.size(0), -1)
            .sum(-1)
            .unsqueeze(-1)
        )

    def mode(self):
        return self.probs.argmax(dim=-1, keepdim=True)


class CategoricalNet(nn.Module):
    def __init__(self, num_inputs: int, num_outputs: int) -> None:
        super().__init__()

        self.linear = nn.Linear(num_inputs, num_outputs)

        nn.init.orthogonal_(self.linear.weight, gain=0.01)
        nn.init.constant_(self.linear.bias, 0)

    def forward(self, x: Tensor) -> CustomFixedCategorical:
        x = self.linear(x)
        return CustomFixedCategorical(logits=x)

    
class ImageNavHFNetPolicyQ(nn.Module):
    def __init__(
        self
        #hidden_size=512,
    ):
        super().__init__()
        #print("HI")
        self.net = ImageNavHFNetQ()
        self.dim_actions = 3
        #self.hidden_size = hidden_size

        self.action_distribution_glob = CategoricalNet(
            self.net.output_size, self.dim_actions
        )
        self.action_distribution_local = CategoricalNet(
            self.net.output_size, self.dim_actions
        )

    def act(
        self,
        observations,
        #rnn_hidden_states,
        #prev_actions,
        #masks,
        deterministic=False,
        debug=False,
    ):
        with torch.no_grad():
            #len_hid_states = int(rnn_hidden_states.shape[-1]/2)

            #assert len_hid_states == self.hidden_size, 'hidden state size error occurred! : '+str(len_hid_states)+' and '+str(self.hidden_size)

            #rnn_hidden_states_glob = rnn_hidden_states[:,:,:len_hid_states]
            #rnn_hidden_states_local = torch.zeros_like(rnn_hidden_states[:,:,(len_hid_states):]) ###
            features_glob, _w_glob = self.net.forward_glob( #, matching = self.net(
                observations#, rnn_hidden_states_glob, prev_actions, masks
            )

            features_local, w_glob, n_matching = self.net.forward_local( #, matching = self.net(
                observations, debug #, rnn_hidden_states_local, prev_actions, masks
            )

        #rnn_hidden_states = torch.cat([rnn_hidden_states_glob,rnn_hidden_states_local], dim=-1)
        distribution_glob = self.action_distribution_glob.linear(features_glob)
        distribution_local = self.action_distribution_local.linear(features_local)

        w_glob = (_w_glob + w_glob) / 2
        w_glob = w_glob.unsqueeze(-1)
        logits = w_glob*distribution_glob + (1.0-w_glob)*distribution_local #distribution_glob
        return logits, n_matching
    
class ImageNavHFNetQ(nn.Module, metaclass=abc.ABCMeta):
    #Q version of ImagenavHFNet, currently doesn't use RNN but might in the future.
    

    def __init__(self):
        super().__init__()
        #self._hidden_size = hidden_size

        self.config = {

            'superglue': {
                'descriptor_dim': 256,
                'keypoint_encoder': [32, 64], #128], # 256],
                'weights': 'indoor',
                'sinkhorn_iterations': 0, #20,
                'match_threshold': 0.2,
                'GNN_layers': ['cross']
            }
        }

        self.fc0 = torch.nn.Linear(256,64)
        self.fc1 = torch.nn.Linear(64,3)
        self.fc2 = torch.nn.Linear(8,128)
        self.fc3 = torch.nn.Linear(128,256)
        #self.state_encoder = build_rnn_state_encoder(
        #    256,
        #    self._hidden_size
        #)
       
        self.fully_connected_glob_1 = torch.nn.Linear(4096, 256)
        self.fully_connected_glob_2 = torch.nn.Linear(512, 256)
        self.sigmoid = torch.nn.Sigmoid()
        #self.state_encoder_glob = build_rnn_state_encoder(
        #    256,
        #    self._hidden_size
        #)
        self.cos = torch.nn.CosineSimilarity(dim=1, eps=1e-6)
        
        self.train()

    @property
    def output_size(self):
        #return self._hidden_size
        return 256

    @property
    def is_blind(self):
        return self.visual_encoder.is_blind
    
    #@property
    #def num_recurrent_layers(self):
    #    return self.state_encoder.num_recurrent_layers #+ self.state_encoder_glob.num_recurrent_layers
    
    def normalize_keypoints(self, kpts, image_shape):
        """ Normalize keypoints locations based on image image_shape"""
        _, _, height, width = image_shape
        one = kpts.new_tensor(1)
        size = torch.stack([one*width, one*height])[None]
        center = size / 2
        scaling = size.max(1, keepdim=True).values * 0.7
        return (kpts - center[:, None, :]) / scaling[:, None, :]          
    
    def forward(self, observations):
        return forward_glob(observations)

    def forward_glob(self, observations):
        
        desc1 = observations['obs_desc_glob']
        desc0 = observations['targ_desc_glob']
        #print(desc0)
        #print(desc0.shape)
        cos_dist = (1.0 - self.cos(desc0, desc1)) / 2.0
        
        desc0 = self.fully_connected_glob_1(desc0)
        desc1 = self.fully_connected_glob_1(desc1)        
        
        x = torch.cat([desc0, desc1], dim=1)
        x_ = self.fully_connected_glob_2(x)
        #x_, rnn_hidden_states = self.state_encoder_glob(x, rnn_hidden_states, masks)


        
        return x_, cos_dist
        


    def forward_local(self, observations, debug=False):

        desc1, kpts1_, score1 = observations['obs_desc'], observations['obs_det'], observations['obs_score']
        desc0, kpts0_, score0 = observations['targ_desc'], observations['targ_det'], observations['targ_score']
        
        # Keypoint normalization.
        kpts0 = self.normalize_keypoints(kpts0_, observations['rgb'].permute(0,3,1,2).shape) 
        kpts1 = self.normalize_keypoints(kpts1_, observations['rgb'].permute(0,3,1,2).shape)
        
        descs = []
        n_match = []
        assert kpts1.shape[0] == desc1.shape[0], 'batch number error!'
        for i in range(kpts1.shape[0]): # for each batch
            # matching
            matches, mkpts0, mkpts1, mconf = match_descriptors_gpu(desc0[i], desc1[i], max_ratio=0.8)
            
            if debug is True:
                #print(desc0[i].shape,desc1[i].shape)
                #print(matches)
                #print(mkpts0)
                #print(mkpts1)
                #print(mconf)
                color = cm.jet(mconf.cpu().numpy())
                text = [
                    'Keypoints: {}:{}'.format(len(kpts0.cpu().numpy()[0]), len(kpts1.cpu().numpy()[0])),
                    'Matches: {}'.format(len(mkpts0)),
                ]

                make_matching_plot(
                    observations["targ_rgb"]/255., observations["rgb"].cpu().numpy()[0,:,:,:]/255., kpts0_.cpu().numpy()[i], kpts1_.cpu().numpy()[0-i],
                     kpts0_.cpu().numpy()[i][mkpts0], kpts1_.cpu().numpy()[i][mkpts1], color,
                    text, 'matching'+str(time.time())+'.png', True,
                    False, True)                
            
            n_match.append(len(mkpts0))
            if len(mkpts0) == 0:
                descs.append(torch.zeros(256).to(observations['rgb'].device))
            else:
                # matching displacement
                del_kpts = kpts0[i][mkpts0] - kpts1[i][mkpts1]
                
                desc_ = self.fc1(self.fc0(desc1[i][mkpts1]))
                desc = torch.cat([kpts1[i][mkpts1], del_kpts, score1[i][mkpts1].unsqueeze(-1), desc_], dim=-1)
                desc = self.fc3(self.fc2(desc))
                
                descs.append(torch.mean(desc,dim=0))

        graph_embedding_0 = torch.stack(descs, dim=0)
        n_match = np.array(n_match)
        if debug is True:
            print("n_match: ", n_match)
        n_match = (n_match >= 5).astype(np.float32) * n_match
        cos_dist = 1.0 - torch.Tensor(n_match)/48.0

        cos_dist = cos_dist.to(observations['rgb'].device)
    
        x = graph_embedding_0
        #x, rnn_hidden_states = self.state_encoder(x, rnn_hidden_states, masks)

        return x, cos_dist, n_match


def plot_image_pair(imgs, dpi=100, size=6, pad=.5):
    n = len(imgs)
    assert n == 2, 'number of images must be two'
    figsize = (size*n, size*3/4) if size is not None else None
    _, ax = plt.subplots(1, n, figsize=figsize, dpi=dpi)
    for i in range(n):
        # ax[i].imshow(imgs[i], cmap=plt.get_cmap('gray'), vmin=0, vmax=255)
        ax[i].imshow(imgs[i])
        ax[i].get_yaxis().set_ticks([])
        ax[i].get_xaxis().set_ticks([])
        for spine in ax[i].spines.values():  # remove frame
            spine.set_visible(False)
    plt.tight_layout(pad=pad)


def plot_keypoints(kpts0, kpts1, color='w', ps=2):
    ax = plt.gcf().axes
    ax[0].scatter(kpts0[:, 0], kpts0[:, 1], c=color, s=ps)
    ax[1].scatter(kpts1[:, 0], kpts1[:, 1], c=color, s=ps)


def plot_matches(kpts0, kpts1, color, lw=1.5, ps=4):
    fig = plt.gcf()
    ax = fig.axes
    fig.canvas.draw()

    transFigure = fig.transFigure.inverted()
    fkpts0 = transFigure.transform(ax[0].transData.transform(kpts0))
    fkpts1 = transFigure.transform(ax[1].transData.transform(kpts1))

    fig.lines = [matplotlib.lines.Line2D(
        (fkpts0[i, 0], fkpts1[i, 0]), (fkpts0[i, 1], fkpts1[i, 1]), zorder=1,
        transform=fig.transFigure, c=color[i], linewidth=lw)
                 for i in range(len(kpts0))]
    ax[0].scatter(kpts0[:, 0], kpts0[:, 1], c=color, s=ps)
    ax[1].scatter(kpts1[:, 0], kpts1[:, 1], c=color, s=ps)

def make_matching_plot(image0, image1, kpts0, kpts1, mkpts0, mkpts1,
                       color, text, path, show_keypoints=False,
                       fast_viz=False, display=False,
                       opencv_title='matches', small_text=[]):

    if fast_viz:
        make_matching_plot_fast(image0, image1, kpts0, kpts1, mkpts0, mkpts1,
                                color, text, path, show_keypoints, 10,
                                display, opencv_title, small_text)
        return

    plot_image_pair([image0, image1])
    if show_keypoints:
        plot_keypoints(kpts0, kpts1, color='k', ps=4)
        plot_keypoints(kpts0, kpts1, color='w', ps=2)
    plot_matches(mkpts0, mkpts1, color)

    fig = plt.gcf()
    txt_color = 'k' if image0[:100, :150].mean() > 200 else 'w'
    fig.text(
        0.01, 0.99, '\n'.join(text), transform=fig.axes[0].transAxes,
        fontsize=15, va='top', ha='left', color=txt_color)

    txt_color = 'k' if image0[-100:, :150].mean() > 200 else 'w'
    fig.text(
        0.01, 0.01, '\n'.join(small_text), transform=fig.axes[0].transAxes,
        fontsize=5, va='bottom', ha='left', color=txt_color)
    if display is True:
        plt.show()
    else:
        plt.savefig(str(path), bbox_inches='tight', pad_inches=0)
    plt.close()


def make_matching_plot_fast(image0, image1, kpts0, kpts1, mkpts0,
                            mkpts1, color, text, path=None,
                            show_keypoints=False, margin=10,
                            opencv_display=False, opencv_title='',
                            small_text=[]):
    H0, W0 = image0.shape
    H1, W1 = image1.shape
    H, W = max(H0, H1), W0 + W1 + margin

    out = 255*np.ones((H, W), np.uint8)
    out[:H0, :W0] = image0
    out[:H1, W0+margin:] = image1
    out = np.stack([out]*3, -1)

    if show_keypoints:
        kpts0, kpts1 = np.round(kpts0).astype(int), np.round(kpts1).astype(int)
        white = (255, 255, 255)
        black = (0, 0, 0)
        for x, y in kpts0:
            cv2.circle(out, (x, y), 2, black, -1, lineType=cv2.LINE_AA)
            cv2.circle(out, (x, y), 1, white, -1, lineType=cv2.LINE_AA)
        for x, y in kpts1:
            cv2.circle(out, (x + margin + W0, y), 2, black, -1,
                       lineType=cv2.LINE_AA)
            cv2.circle(out, (x + margin + W0, y), 1, white, -1,
                       lineType=cv2.LINE_AA)

    mkpts0, mkpts1 = np.round(mkpts0).astype(int), np.round(mkpts1).astype(int)
    color = (np.array(color[:, :3])*255).astype(int)[:, ::-1]
    for (x0, y0), (x1, y1), c in zip(mkpts0, mkpts1, color):
        c = c.tolist()
        cv2.line(out, (x0, y0), (x1 + margin + W0, y1),
                 color=c, thickness=1, lineType=cv2.LINE_AA)
        # display line end-points as circles
        cv2.circle(out, (x0, y0), 2, c, -1, lineType=cv2.LINE_AA)
        cv2.circle(out, (x1 + margin + W0, y1), 2, c, -1,
                   lineType=cv2.LINE_AA)

    # Scale factor for consistent visualization across scales.
    sc = min(H / 640., 2.0)

    # Big text.
    Ht = int(30 * sc)  # text height
    txt_color_fg = (255, 255, 255)
    txt_color_bg = (0, 0, 0)
    for i, t in enumerate(text):
        cv2.putText(out, t, (int(8*sc), Ht*(i+1)), cv2.FONT_HERSHEY_DUPLEX,
                    1.0*sc, txt_color_bg, 2, cv2.LINE_AA)
        cv2.putText(out, t, (int(8*sc), Ht*(i+1)), cv2.FONT_HERSHEY_DUPLEX,
                    1.0*sc, txt_color_fg, 1, cv2.LINE_AA)

    # Small text.
    Ht = int(18 * sc)  # text height
    for i, t in enumerate(reversed(small_text)):
        cv2.putText(out, t, (int(8*sc), int(H-Ht*(i+.6))), cv2.FONT_HERSHEY_DUPLEX,
                    0.5*sc, txt_color_bg, 2, cv2.LINE_AA)
        cv2.putText(out, t, (int(8*sc), int(H-Ht*(i+.6))), cv2.FONT_HERSHEY_DUPLEX,
                    0.5*sc, txt_color_fg, 1, cv2.LINE_AA)

    if path is not None:
        cv2.imwrite(str(path), out)

    if opencv_display:
        cv2.imshow(opencv_title, out)
        cv2.waitKey(1)

    return out


