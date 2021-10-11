import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions.categorical import Categorical
import numpy as np

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

class GRUNet(nn.Module):
    def __init__(self, input_dim, hidden_dim, n_layers):
        super(GRUNet, self).__init__()
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.n_layers = n_layers
        self.gru = nn.GRU(input_dim, hidden_dim, n_layers, batch_first=True)

    def forward(self, x, h):
        out, h = self.gru(x.view(-1, self.n_layers, self.input_dim), h)
        return h

    def init_hidden(self, batch_size):
        weight = next(self.parameters()).data
        hidden = weight.new(self.n_layers, batch_size, self.hidden_dim).zero_()#.cuda()
        return hidden

class Recovery(nn.Module):
    def __init__(self, action_dim, memory_dim, feature_dim, GRU_size):
        super(Recovery, self).__init__()
        self.memory_dim = memory_dim
        self.feature_dim = feature_dim
        self.GRU_size = GRU_size
        self.action_dim = action_dim
        self.eta = torch.Tensor([0.])
        self.GRU = GRUNet(self.memory_dim+self.feature_dim, self.GRU_size, 1)
        self.GRU_attention_fc = FCN_2layer(self.GRU_size, 256, 1, activation='tanh')
        self.GRU_out_fc = FCN_2layer(self.GRU_size, 256, self.action_dim)
        self.h = self.GRU.init_hidden(1)
        
    def forward(self, memories_list, curr_feat):
        path_length = len(memories_list)
        memories_list = torch.cat(memories_list, 0)
        max_eta = torch.Tensor([path_length-1])
        attention = torch.stack([torch.exp(-torch.abs(self.eta - j)) for j in range(path_length)], 0)
        mu = torch.sum(torch.mul(memories_list, attention), dim=0)
        gru_in = torch.cat([curr_feat, mu.view(-1, self.memory_dim)], -1)
        self.h = self.GRU(gru_in, self.h)
        b = 1 + self.GRU_attention_fc(self.h.view(-1, self.GRU_size)).squeeze(0)
        self.eta = torch.min(self.eta + b, max_eta)
        action_pred = self.GRU_out_fc(self.h.view(-1, self.GRU_size))
        action = int(torch.argmax(action_pred).cpu().numpy())
        if action == 0 : # go straight
            return_action = [0., 0.40, 0.] # ['forward'] * 3
        elif action == 1 : # turn left
            return_action = [-30., 0., 0.] # ['turn_left'] * 3
        elif action == 2 : # turn right
            return_action = [30., 0., 0.] # ['right'] * 3
        return return_action




"""
def attention(q,k,v):
    score = F.softmax(torch.matmul(q, k.transpose(2,3)))
    output = torch.matmul(score, v)
    return output, score

class MultiheadAttention(nn.Module):
    def __init__(self, d_model, d_k, d_v, heads=8):
        super().__init__()

        self.d_model = d_model
        self.d_k, self.d_v = d_k, d_v
        self.heads = heads
        self.wq = nn.Linear(d_model, heads * self.d_k, bias=False)
        self.wk = nn.Linear(d_model, heads * self.d_k, bias=False)
        self.wv = nn.Linear(d_model, heads * self.d_v, bias=False)

        self.out = nn.Linear(heads * self.d_v, d_model, bias=False)
        self.layer_norm = nn.LayerNorm(d_model)

    def forward(self, q, k, v):
        B, len_q, len_k, len_v = q.shape[0], q.shape[1], k.shape[1], v.shape[1]

        residual = q
        q = self.layer_norm(q)
        q = self.wq(q).view(B, len_q, self.heads, self.d_k).transpose(1,2)
        k = self.wk(k).view(B, len_k, self.heads, self.d_k).transpose(1,2)
        v = self.wv(v).view(B, len_v, self.heads, self.d_v).transpose(1,2)

        output, score = attention(q, k, v)
        output = output.transpose(1,2).contiguous().view(B, len_q, -1)
        output = self.out(output)
        output += residual.squeeze(2)
        return output, score.sum(1)

class Recovery(nn.Module):
    def __init__(self):
        super(Recovery, self).__init__()
        self.Attention = MultiheadAttention(d_model=256, d_k=256, d_v=256)
        self.fc = nn.Linear(256, 4)
        self.entropy_th = 1.0


    def forward(self, vis_mem, img_feature):
        #- state: state from StateDeterminant Module
        #- img: curreunt image input (doesn't need if visual memory contains the current input image)
        #Output:
        #- action(s) guides to reach previous POI
        
        assert(len(vis_mem.shape) >= 3, 'invalid memory shape') # L * dim
        if isinstance(vis_mem, np.ndarray): vis_mem = torch.from_numpy(vis_mem)
        elif isinstance(vis_mem, list): vis_mem = torch.cat(vis_mem)
        elif isinstance(vis_mem, torch.Tensor): pass
        else: raise Exception('Valid vis_mem type : pytorch tensor, list, numpy array')

        if len(vis_mem.shape) == 2 :
            vis_mem = vis_mem.unsqueeze(0)

        out, attn = self.Attention(q=img_feature.unsqueeze(1), k=vis_mem, v=vis_mem)
        weight = F.softmax(attn)
        entropy = Categorical(probs=weight).entropy()
        if entropy > self.entropy_th : info = False
        else: info = True
        action = self.fc(out)
        act_probs = F.softmax(action)
        return_action = torch.argmax(act_probs)
        done = True if return_action == 0 else False

        if return_action == 0 : # go straight
            return_action = [0., 0.40, 0.] # ['forward'] * 3
        elif return_action == 1 : # turn left
            return_action = [-30., 0., 0.] # ['turn_left'] * 3
        elif return_action == 2 : # turn right
            return_action = [30., 0., 0.] # ['right'] * 3
        elif return_action == 3:  # go back
            return_action = [0., -0.40, 0.] # ['backward'] * 3

        # done: is back home, info: whether visual memory matching succeeded or not
        return return_action, done, info


if __name__=='__main__':
    policy = Recovery()
    L = 15
    vis_mem = torch.rand([15,512])
    img_feature = torch.rand([1,512])
    action, done, info = policy(vis_mem, img_feature)
    print(action, done, info)
"""
