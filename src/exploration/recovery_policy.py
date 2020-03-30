import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions.categorical import Categorical
import numpy as np

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
            return_action = ['forward'] * 3
        elif return_action == 1 : # turn left
            return_action = ['turn_left'] * 3
        elif return_action == 2 : # turn right
            return_action = ['right'] * 3
        elif return_action == 3:  # turn right
            return_action = ['backward'] * 3

        # done: is back home, info: whether visual memory matching succeeded or not
        return return_action, done, info


if __name__=='__main__':
    policy = Recovery()
    L = 15
    vis_mem = torch.rand([15,512])
    img_feature = torch.rand([1,512])
    action, done, info = policy(vis_mem, img_feature)
    print(action, done, info)