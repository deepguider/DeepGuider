from __future__ import print_function
import argparse
from math import log10, ceil
import random
import shutil
import json
from os.path import join, exists, isfile, realpath, dirname
from os import makedirs, remove, chdir, environ

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.autograd import Variable
from torch.utils.data import DataLoader, SubsetRandomSampler
from torch.utils.data.dataset import Subset
import torchvision.transforms as transforms
from PIL import Image
from datetime import datetime
import torchvision.datasets as datasets
import torchvision.models as models
import h5py
import faiss


from visdom import Visdom

from tensorboardX import SummaryWriter
import numpy as np
from netvlad import netvlad

from scipy import io as sio

import cv2 as cv

from ipdb import set_trace as bp
import sys; sys.path.insert(0,'data_vps/netvlad/ccsmmutils'); import img_utils as myiu

class vps:
    def __init__(self):
        self.gps_lat = 0.0 #Latitude
        self.gps_long = 0.0 #Longitude
        self.vps_lat = 0.0 # Latitude from VPS function
        self.vps_long = 0.0 # Longitude from VPS function
        self.angle = -1  # road direction (radian)
        self.vps_prob = -1   # reliablity of the result. 0: fail ~ 1: success
        self.K = 5 # K for knn
        self.ToTensor = transforms.ToTensor()
        self.vps_IDandConf = [0,0]
        self.verbose = False # 1 : print internal results

    def init_param(self):
        self.parser = argparse.ArgumentParser(description='pytorch-NetVlad')
        self.parser.add_argument('--mode', type=str, default='test', help='Mode', choices=['train', 'test', 'cluster'])
        self.parser.add_argument('--batchSize', type=int, default=4, 
                help='Number of triplets (query, pos, negs). Each triplet consists of 12 images.')
        self.parser.add_argument('--cacheRefreshRate', type=int, default=1000, 
                help='How often to refresh cache, in number of queries. 0 for off')
        self.parser.add_argument('--nEpochs', type=int, default=30, help='number of epochs to train for')
        self.parser.add_argument('--start-epoch', default=0, type=int, metavar='N', 
                help='manual epoch number (useful on restarts)')
        self.parser.add_argument('--optim', type=str, default='SGD', help='optimizer to use', choices=['SGD', 'ADAM'])
        self.parser.add_argument('--lr', type=float, default=0.0001, help='Learning Rate.')
        self.parser.add_argument('--lrStep', type=float, default=5, help='Decay LR ever N steps.')
        self.parser.add_argument('--lrGamma', type=float, default=0.5, help='Multiply LR by Gamma for decaying.')
        self.parser.add_argument('--weightDecay', type=float, default=0.001, help='Weight decay for SGD.')
        self.parser.add_argument('--momentum', type=float, default=0.9, help='Momentum for SGD.')
        self.parser.add_argument('--nocuda', action='store_true', help='Dont use cuda')
        self.parser.add_argument('--threads', type=int, default=8, help='Number of threads for each data loader to use')
        self.parser.add_argument('--seed', type=int, default=123, help='Random seed to use.')
        self.parser.add_argument('--dataPath', type=str, default='netlvad/netvlad_v100_datasets/', help='Path for centroid data.')
        self.parser.add_argument('--runsPath', type=str, default='netvlad/checkpoints/runs/', help='Path to save runs to.')
        self.parser.add_argument('--savePath', type=str, default='checkpoints', 
                help='Path to save checkpoints to in logdir. Default=netvlad/checkpoints/')
#        self.parser.add_argument('--cachePath', type=str, default=environ['TMPDIR'], help='Path to save cache to.')
        self.parser.add_argument('--cachePath', type=str, default='/tmp', help='Path to save cache to.')
#        self.parser.add_argument('--resume', type=str, default='netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint_gpu4', help='Path to load checkpoint from, for resuming training or testing.')
        self.parser.add_argument('--ckpt', type=str, default='latest', 
                help='Resume from latest or best checkpoint.', choices=['latest', 'best'])
        self.parser.add_argument('--evalEvery', type=int, default=1, 
                help='Do a validation set run, and save, every N epochs.')
        self.parser.add_argument('--patience', type=int, default=10, help='Patience for early stopping. 0 is off.')
        self.parser.add_argument('--arch', type=str, default='vgg16', 
                help='basenetwork to use', choices=['vgg16', 'alexnet'])
        self.parser.add_argument('--vladv2', action='store_true', help='Use VLAD v2')
        self.parser.add_argument('--pooling', type=str, default='netvlad', help='type of pooling to use',
                choices=['netvlad', 'max', 'avg'])
        self.parser.add_argument('--num_clusters', type=int, default=64, help='Number of NetVlad clusters. Default=64')
        self.parser.add_argument('--margin', type=float, default=0.1, help='Margin for triplet loss. Default=0.1')
        self.parser.add_argument('--split', type=str, default='val', help='Data split to use for testing. Default is val', 
                choices=['test', 'test250k', 'train', 'val'])
        self.parser.add_argument('--fromscratch', action='store_true', help='Train from scratch rather than using pretrained models')


        ######(begin) Following defaults are combination of 9run_vps_ccsmm.sh
        self.parser.add_argument('--nGPU', type=int, default=1, help='number of GPU to use.')
        self.parser.add_argument('--resume', type=str, default='data_vps/netvlad/pretrained_checkpoint/vgg16_netvlad_checkpoint', help='Path to load checkpoint from, for resuming training or testing.')
        #self.parser.add_argument('--dataset', type=str, default='pittsburgh', help='Dataset to use', choices=['pittsburgh','deepguider'])
        self.parser.add_argument('--dataset', type=str, default='deepguider', help='Dataset to use', choices=['pittsburgh','deepguider'])
        self.parser.add_argument('--cacheBatchSize', type=int, default=4, help='Batch size for caching and testing')

        self.parser.add_argument('--dbFeat_fname', type=str, default='data_vps/prebuilt_dbFeat.mat', help='dbFeat file calculated in advance')
        #self.parser.add_argument('--dbFeat_fname', type=str, default='prebuilt_dbFeat.mat', help='dbFeat file calculated in advance')
        self.parser.add_argument('--qFeat_fname', type=str, default='data_vps/prebuilt_qFeat.mat', help='dbFeat file calculated in advance')
        #self.parser.add_argument('--save_dbFeat', default=False, action='store_true', help='Save dbFeat')
        self.parser.add_argument('--save_dbFeat', default=True, action='store_true', help='Save dbFeat')
        #self.parser.add_argument('--save_qFeat', default=False, action='store_true', help='Save qFeat')
        #self.parser.add_argument('--use_saved_dbFeat', default=False, action='store_true', help='Use save dbFeat feature which is calucated in adavnce') #default

        self.parser.add_argument('--verbose', default=False, action='store_true', help='Print internal messages')
        ######(end) Following defaults are combination of 9run_vps_ccsmm.sh

        return 1


    def initialize(self):
        self.init_param()
        opt = self.parser.parse_args()
        self.verbose = opt.verbose
        restore_var = ['lr', 'lrStep', 'lrGamma', 'weightDecay', 'momentum', 
                'runsPath', 'savePath', 'arch', 'num_clusters', 'pooling', 'optim',
                'margin', 'seed', 'patience']
        if opt.resume:
            flag_file = join(opt.resume, 'checkpoints', 'flags.json')
            if exists(flag_file):
                with open(flag_file, 'r') as f:
                    stored_flags = {'--'+k : str(v) for k,v in json.load(f).items() if k in restore_var}
                    to_del = []
                    for flag, val in stored_flags.items():
                        for act in self.parser._actions:
                            if act.dest == flag[2:]:
                                # store_true / store_false args don't accept arguments, filter these 
                                if type(act.const) == type(True):
                                    if val == str(act.default):
                                        to_del.append(flag)
                                    else:
                                        stored_flags[flag] = ''
                    for flag in to_del: del stored_flags[flag]
    
                    train_flags = [x for x in list(sum(stored_flags.items(), tuple())) if len(x) > 0]
                    print('Restored flags:', train_flags)
                    opt = self.parser.parse_args(train_flags, namespace=opt)
    
        print(opt)
    
        cuda = not opt.nocuda
        if cuda and not torch.cuda.is_available():
            raise Exception("No GPU found, please run with --nocuda")
    
        device = torch.device("cuda" if cuda else "cpu")
    
        random.seed(opt.seed)
        np.random.seed(opt.seed)
        torch.manual_seed(opt.seed)
        if cuda:
            torch.cuda.manual_seed(opt.seed)
    
        print('===> Building model begin')
    
        pretrained = not opt.fromscratch
        if opt.arch.lower() == 'alexnet':
            self.encoder_dim = 256
            encoder = models.alexnet(pretrained=pretrained)
            # capture only features and remove last relu and maxpool
            layers = list(encoder.features.children())[:-2]
    
            if pretrained:
                # if using pretrained only train conv5
                for l in layers[:-1]:
                    for p in l.parameters():
                        p.requires_grad = False
    
        elif opt.arch.lower() == 'vgg16':
            self.encoder_dim = 512
            encoder = models.vgg16(pretrained=pretrained)
            # capture only feature part and remove last relu and maxpool
        
            layers = list(encoder.features.children())[:-2]
    
            if pretrained:
                # if using pretrained then only train conv5_1, conv5_2, and conv5_3
                for l in layers[:-5]: 
                    for p in l.parameters():
                        p.requires_grad = False
    
        if opt.mode.lower() == 'cluster' and not opt.vladv2:
            layers.append(L2Norm())
    
        encoder = nn.Sequential(*layers)
        model = nn.Module() 
        model.add_module('encoder', encoder)
    
   
        if opt.mode.lower() != 'cluster':
            if opt.pooling.lower() == 'netvlad':
                net_vlad = netvlad.NetVLAD(num_clusters=opt.num_clusters, dim=self.encoder_dim, vladv2=opt.vladv2)
                if not opt.resume: 
                    if opt.mode.lower() == 'train':
                        initcache = join(opt.dataPath, 'centroids', opt.arch + '_' + train_set.dataset + '_' + str(opt.num_clusters) +'_desc_cen.hdf5')
                    else:
                        initcache = join(opt.dataPath, 'centroids', opt.arch + '_' + whole_test_set.dataset + '_' + str(opt.num_clusters) +'_desc_cen.hdf5')
    
    
                    if not exists(initcache):
                        raise FileNotFoundError('Could not find clusters, please run with --mode=cluster before proceeding')
    
                    with h5py.File(initcache, mode='r') as h5: 
                        clsts = h5.get("centroids")[...]
                        traindescs = h5.get("descriptors")[...]
                        net_vlad.init_params(clsts, traindescs) 
                        del clsts, traindescs
    
                model.add_module('pool', net_vlad)
            elif opt.pooling.lower() == 'max':
                global_pool = nn.AdaptiveMaxPool2d((1,1))
                model.add_module('pool', nn.Sequential(*[global_pool, Flatten(), L2Norm()]))
            elif opt.pooling.lower() == 'avg':
                global_pool = nn.AdaptiveAvgPool2d((1,1))
                model.add_module('pool', nn.Sequential(*[global_pool, Flatten(), L2Norm()]))
            else:
                raise ValueError('Unknown pooling type: ' + opt.pooling)

        isParallel = False
        if opt.nGPU > 1 and torch.cuda.device_count() > 1:
            model.encoder = nn.DataParallel(model.encoder)
            if opt.mode.lower() != 'cluster':
                model.pool = nn.DataParallel(model.pool)
            isParallel = True
    
        if not opt.resume:
            model = model.to(device)
        
        if opt.mode.lower() == 'train':
            if opt.optim.upper() == 'ADAM':
                optimizer = optim.Adam(filter(lambda p: p.requires_grad, 
                    model.parameters()), lr=opt.lr)#, betas=(0,0.9))
            elif opt.optim.upper() == 'SGD':
                optimizer = optim.SGD(filter(lambda p: p.requires_grad, 
                    model.parameters()), lr=opt.lr,
                    momentum=opt.momentum,
                    weight_decay=opt.weightDecay)
    
                scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=opt.lrStep, gamma=opt.lrGamma)
            else:
                raise ValueError('Unknown optimizer: ' + opt.optim)
    
            # original paper/code doesn't sqrt() the distances, we do, so sqrt() the margin, I think :D
            criterion = nn.TripletMarginLoss(margin=opt.margin**0.5, 
                    p=2, reduction='sum').to(device)
    
        if opt.resume:
            if opt.ckpt.lower() == 'latest':
                resume_ckpt = join(opt.resume, 'checkpoints', 'checkpoint.pth.tar')
            elif opt.ckpt.lower() == 'best':
                resume_ckpt = join(opt.resume, 'checkpoints', 'model_best.pth.tar')
    
            if isfile(resume_ckpt):
                print("=> loading checkpoint '{}'".format(resume_ckpt))
                checkpoint = torch.load(resume_ckpt, map_location=lambda storage, loc: storage)
                opt.start_epoch = checkpoint['epoch']
                best_metric = checkpoint['best_score']
                model.load_state_dict(checkpoint['state_dict'])
                model = model.to(device)
                if opt.mode == 'train':
                    optimizer.load_state_dict(checkpoint['optimizer'])
                print("=> loaded checkpoint '{}' (epoch {})"
                      .format(resume_ckpt, checkpoint['epoch']))
            else:
                print("=> no checkpoint found at '{}'".format(resume_ckpt))
    
        self.model = model
        print('===> Building model end(vps.py)')

        return 1 # Non-zero means success return 
    

    def test_sub(self,eval_set,epoch=0):
        opt = self.parser.parse_args()
        cuda = not opt.nocuda
        device = torch.device("cuda" if cuda else "cpu")

        test_data_loader = DataLoader(dataset=eval_set, 
                    num_workers=opt.threads, batch_size=opt.cacheBatchSize, shuffle=False, 
                    pin_memory=cuda)
    
        self.model.eval()

        with torch.no_grad():
            print('====> Extracting Features')
            pool_size = self.encoder_dim
            if opt.pooling.lower() == 'netvlad': pool_size *= opt.num_clusters
            Feat = np.empty((len(eval_set), pool_size))
        
            for iteration, (input, indices) in enumerate(test_data_loader, 1):
                input = input.to(device) #[24, 3, 480, 640]
                image_encoding = self.model.encoder(input) #[24, 512, 30, 40]
                vlad_encoding = self.model.pool(image_encoding) #[24,32768] 
        
                #Feat : [17608, 32768]
                Feat[indices.detach().numpy(), :] = vlad_encoding.detach().cpu().numpy() #[24,32768]
                if self.verbose:
                    try:
                        if (iteration-1) % 200 == 0:
                            print("==> Batch ({}/{})".format(iteration,len(test_data_loader)), flush=True)
                            avp = self.GAP1dChannel(image_encoding)
                            myiu.clf()
                            myiu.imshow(input,221,'input')
                            myiu.imshow(-avp*0.1,222,'encoding')
                            myiu.plot(vlad_encoding,223,'vlad')
                    except:
                        print("Cannot Display")
            del input, image_encoding, vlad_encoding
        del test_data_loader
        return Feat
    
    def GAP1dChannel(self,img):
        n,c,w,h = img.shape
        img1 = img.view(n,c,w*h).permute(0,2,1) #[n,w*h,c]
        tnn  = nn.AvgPool1d(kernel_size=c)
        img2 = tnn(img1) #[n,w*h,1]
        img = img2.permute(0,2,1).view(n,1,w,h) #[n,1,w,h]
        img = img.repeat(1,3,1,1) #[n,3,w,h]
        return img


    def test_dg(self,eval_set_db,eval_set_q, epoch=0, write_tboard=False):
        # TODO what if features dont fit in memory? 
        opt = self.parser.parse_args()
        cuda = not opt.nocuda

        if opt.save_dbFeat:
            # extracted for db, now split in own sets
            dbFeat = self.test_sub(eval_set_db,epoch=epoch)
            dbFeat = dbFeat.astype('float32') #[ndbImg,32768]
            dbFeat_dict={'Feat':dbFeat}
            sio.savemat(opt.dbFeat_fname,dbFeat_dict)
        else:
            dbFeat = sio.loadmat(opt.dbFeat_fname)
            dbFeat = dbFeat['Feat']
            dbFeat = np.ascontiguousarray(dbFeat)


        # extracted for query, now split in own sets
        qFeat = self.test_sub(eval_set_q,epoch=epoch)
        qFeat = qFeat.astype('float32') #[nqImg,32768]

        #if opt.save_qFeat:
        #    # extracted for query, now split in own sets
        #    qFeat = self.test_sub(eval_set_q,epoch=epoch)
        #    qFeat = qFeat.astype('float32') #[nqImg,32768]
        #    qFeat_dict={'Feat':qFeat}
        #    sio.savemat(opt.qFeat_fname,qFeat_dict)
        #else:
        #    qFeat = sio.loadmat(opt.qFeat_fname)
        #    qFeat = qFeat['Feat']
        #    qFeat = np.ascontiguousarray(qFeat)


        test_db_data_loader = DataLoader(dataset=eval_set_db, 
                    num_workers=opt.threads, batch_size=opt.cacheBatchSize, shuffle=False, 
                    pin_memory=cuda)

        test_q_data_loader = DataLoader(dataset=eval_set_q, 
                    num_workers=opt.threads, batch_size=opt.cacheBatchSize, shuffle=False, 
                    pin_memory=cuda)

        
        print('====> Building faiss index')
        pool_size = dbFeat[-1].size
        faiss_index = faiss.IndexFlatL2(pool_size) #32768
        faiss_index.add(dbFeat)
    
        print('====> Calculating recall @',self.K)
        #n_values = [1,5,10,20] #n nearest neighbors
    
        pred_L2dist, pred_idx = faiss_index.search(qFeat, self.K) #predictions : [7608,1]

        if self.verbose:
            print('predicted ID:\n', pred_idx)

        qImage  = test_q_data_loader.dataset.ImgStruct.Image
        dbImage = test_db_data_loader.dataset.ImgStruct.Image
        dbImage_predicted = dbImage[pred_idx[:,0]] #Use first K for display

        self.qImage = qImage
        self.dbImage = dbImage
        self.pred_idx = pred_idx
        self.pred_L2dist = pred_L2dist

        import os
        if self.verbose:
            print('QueryImage <=================> predicted dbImage')
        match_cnt = 0
        total_cnt = len(qImage)
        for i in range(total_cnt):
            qName = os.path.basename(qImage[i].item()).strip()
            dbName_predicted = os.path.basename(dbImage_predicted[i].item()).strip()
        #    IDs = ['spherical_2812920067800000','spherical_2812920067800000']
            lat,lon,deg = self.ID2LL(self.Fname2ID(dbName_predicted))

            if qName in 'newquery.jpg':
                flist = dbImage[pred_idx[i]]
                vps_imgID = self.Fname2ID(flist)
                vps_imgConf = [val for val in pred_L2dist[i]]
                vps_imgID = [int(i) for i in vps_imgID]
                vps_imgConf = [float(1/i) for i in vps_imgConf]
                self.vps_IDandConf = [vps_imgID, vps_imgConf]

            if self.verbose:
                if self.Fname2ID(qName)[0] in self.Fname2ID(dbName_predicted)[0]:
                    match_cnt = match_cnt + 1
                    print('[Q]',qName,'<==> [Pred]', dbName_predicted,'[Lat,Lon] =',lat,',',lon,'[*Matched]')
                else:
                    print('[Q]',qName,'<==> [Pred]', dbName_predicted,'[Lat,Lon] =',lat,',',lon)

        acc = match_cnt/total_cnt
        if self.verbose:
            print('Accuracy : {} / {} = {} % in {} DB images'.format(match_cnt,total_cnt,acc*100.0,len(dbImage)))

        print('Return from vps.py->apply()')

        return acc


    def test(self,eval_set, epoch=0, write_tboard=False):
        # TODO what if features dont fit in memory? 
        opt = self.parser.parse_args()
        cuda = not opt.nocuda
        device = torch.device("cuda" if cuda else "cpu")

        test_data_loader = DataLoader(dataset=eval_set, 
                    num_workers=opt.threads, batch_size=opt.cacheBatchSize, shuffle=False, 
                    pin_memory=cuda)
    
        self.model.eval()

        qImage = test_data_loader.dataset.dbStruct.qImage
        dbImage = test_data_loader.dataset.dbStruct.dbImage

        with torch.no_grad():
            print('====> Extracting Features')
            pool_size = self.encoder_dim
            if opt.pooling.lower() == 'netvlad': pool_size *= opt.num_clusters
            dbqFeat = np.empty((len(eval_set), pool_size))
        
            for iteration, (input, indices) in enumerate(test_data_loader, 1):
                input = input.to(device) #[24, 3, 480, 640]
#                print(indices,input.shape)
#                if indices < eval_set.dbStruct.numDb:
#                    print(dbImage[indices],input.shape[-1])
#                else:
#                    print(qImage[indices-eval_set.dbStruct.numDb],input.shape[-1])
#                continue
                image_encoding = self.model.encoder(input) #[24, 512, 30, 40]
                vlad_encoding = self.model.pool(image_encoding) #[24,32768] 
        
                #dbqFeat : [17608, 32768]
                dbqFeat[indices.detach().numpy(), :] = vlad_encoding.detach().cpu().numpy() #[24,32768]
                try:
                    if iteration % 50 == 0 or len(test_data_loader) <= 10:
                        print("==> Batch ({}/{})".format(iteration,len(test_data_loader)), flush=True)
                        myiu.clf()
                        myiu.imshow(input,221,'input')
                        myiu.imshow(image_encoding[:,:3,:,:],222,'encoding')
                        myiu.plot(vlad_encoding,223,'vlad')
                    if iteration*opt.cacheBatchSize >= eval_set.dbStruct.numDb:
                        myiu.clf()
                        myiu.imshow(input[-1,:,:,:],221,'input')
                        myiu.imshow(image_encoding[-1,:3,:,:],222,'encoding')
                        myiu.plot(vlad_encoding[-1],223,'vlad')
                except:
                    print("Cannot Display")
            del input, image_encoding, vlad_encoding
        del test_data_loader
    
        if opt.save_dbFeat:
            # extracted for db, now split in own sets
            dbFeat = dbqFeat[:eval_set.dbStruct.numDb].astype('float32') #[10000,32768]
            dbFeat_dict={'dbFeat':dbFeat}
            sio.savemat(opt.dbFeat_fname,dbFeat_dict)
        else:
            dbFeat = sio.loadmat(opt.dbFeat_fname)
            dbFeat = dbFeat['dbFeat']


        # extracted for query, now split in own sets
        qFeat = dbqFeat[eval_set.dbStruct.numDb:].astype('float32') #[7608,32768]
        qFeat_dict={'qFeat':qFeat}

        #if opt.save_qFeat:
        #    # extracted for query, now split in own sets
        #    qFeat = dbqFeat[eval_set.dbStruct.numDb:].astype('float32') #[7608,32768]
        #    qFeat_dict={'qFeat':qFeat}
        #    sio.savemat(opt.qFeat_fname,qFeat_dict)
        #else:
        #    qFeat = sio.loadmat(opt.qFeat_fname)
        #    qFeat = qFeat['qFeat']

        print('====> Building faiss index')
        #qFeat  : [7608,32768], pool_size = 32768 as dimension of feature
        #dbFeat : [10000,32768]
        faiss_index = faiss.IndexFlatL2(pool_size) #32768
        faiss_index.add(dbFeat)
    
        print('====> Calculating recall @ N')
        n_values = [1,5,10,20] #n nearest neighbors
    
        #we want to see 20 nearnest neighbors using following search command.
#        _, predictions = faiss_index.search(qFeat, max(n_values)) #predictions : [7608,20]
    
#        # for each query get those within threshold distance
#        gt = eval_set.getPositives() #Ground Truth
#    
#        correct_at_n = np.zeros(len(n_values))
#        #TODO can we do this on the matrix in one go?
#        for qIx, pred in enumerate(predictions):
#            for i,n in enumerate(n_values):
#                # if in top N then also in top NN, where NN > N
#                if np.any(np.in1d(pred[:n], gt[qIx])):
#                    correct_at_n[i:] += 1
#                    break
#        recall_at_n = correct_at_n / eval_set.dbStruct.numQ
#    
#        recalls = {} #make dict for output
#        for i,n in enumerate(n_values):
#            recalls[n] = recall_at_n[i]
#            print("====> Recall@{}: {:.4f}".format(n, recall_at_n[i]))
#            if write_tboard: writer.add_scalar('Val/Recall@' + str(n), recall_at_n[i], epoch)

        pred_L2dist, pred_idx = faiss_index.search(qFeat, self.K) #predictions : [7608,1]

        print('predicted ID:\n', pred_idx)
        test_data_loader = DataLoader(dataset=eval_set, 
                    num_workers=opt.threads, batch_size=opt.cacheBatchSize, shuffle=False, 
                    pin_memory=cuda)

        qImage = test_data_loader.dataset.dbStruct.qImage
        dbImage = test_data_loader.dataset.dbStruct.dbImage
        dbImage_predicted = test_data_loader.dataset.dbStruct.dbImage[pred_idx[:,0]] #Use first K for display


        import os
        print('QueryImage <=================> predicted dbImage')
        match_cnt = 0
        total_cnt = len(qImage)
        for i in range(total_cnt):
            qName = os.path.basename(qImage[i].item()).strip()
            dbName_predicted = os.path.basename(dbImage_predicted[i].item()).strip()
        #    IDs = ['spherical_2812920067800000','spherical_2812920067800000']
            lat,lon,deg = self.ID2LL(self.Fname2ID(dbName_predicted))

            if qName in 'newquery.jpg':
                flist = test_data_loader.dataset.dbStruct.dbImage[pred_idx[i]]
                vps_imgID = self.Fname2ID(flist)
                vps_imgConf = [val for val in pred_L2dist[i]]
                #self.vps_IDandConf = [vps_imgID, vps_imgConf] #ori
                self.vps_IDandConf = [123456, 1.33] #dbg

            if self.Fname2ID(qName)[0] in self.Fname2ID(dbName_predicted)[0]:
                match_cnt = match_cnt + 1
                print('[Q]',qName,'<==> [Pred]', dbName_predicted,'[Lat,Lon] =',lat,',',lon,'[*Matched]')
            else:
                print('[Q]',qName,'<==> [Pred]', dbName_predicted,'[Lat,Lon] =',lat,',',lon)

        acc = match_cnt/total_cnt
        print('Accuracy : {} / {} = {} % in {} DB images'.format(match_cnt,total_cnt,acc*100.0,len(dbImage)))

#        print('You can investigate the internal data of result here. If you want to exit anyway, press Ctrl-D')
#        return recalls
        return acc

        
    def get_param(self):
        # get parameter
        return 0
        
    def set_param(self):
        # set parameter
        return 0

    def apply(self, image=None, K = 5, gps_lat=None, gps_long=None, gps_accuracy=None, timestamp=None):
        if gps_lat is None:
            self.gps_lat = -1
        if gps_long is None:
            self.gps_long = -1
        if gps_accuracy is None:
            self.gps_accuracy = -1
        if timestamp is None:
            self.timestamp = -1

        self.K = int(K)
        opt = self.parser.parse_args()

        ##### Process Input #####
#        cv.imshow("sample", self.image)
#        cv.waitKey()
#        cv.destroyWindow("sample")

        print('===> Loading dataset(s)')
        epoch = 1

        if opt.dataset.lower() == 'pittsburgh':
            from netvlad import pittsburgh as dataset
            whole_test_set = dataset.get_whole_test_set()
            print('===> Evaluating on test set')
            print('===> Running evaluation step')
            recalls = self.test(whole_test_set, epoch, write_tboard=False)

        elif opt.dataset.lower() == 'deepguider':
            if image is not None:
                cv.imwrite('data_vps/netvlad_etri_datasets/qImg/999_newquery/newquery.jpg',image)
            from netvlad import etri_dbloader as dataset
            dbDir = 'dbImg'
            qDir = 'qImg'
            whole_db_set,whole_q_set = dataset.get_dg_test_set(dbDir,qDir)
            print('===> With Query captured near the ETRI Campus')
            print('===> Evaluating on test set')
            print('===> Running evaluation step')
            recalls = self.test_dg(whole_db_set,whole_q_set, epoch, write_tboard=False)
        else:
            raise Exception('Unknown dataset')

        ##### Results #####
        return self.vps_IDandConf
#        return self.vps_lat, self.vps_long, self.vps_prob, self.gps_lat, self.gps_long

    def getPosVPS(self):
        return self.GPS_Latitude, self.GPS_Longitude

    def getPosVPS(self):
        return self.VPS_Latitude, self.VPS_Longitude

    def getAngle(self):
        return self.angle

    def getProb(self):
        return self.prob

    def Fname2ID(self,flist):
        import os

        if type(flist) is str:
            flist = [flist]
        ID = []
        fcnt = len(flist)
        for i in range(fcnt):
            imgID = os.path.basename(flist[i]).strip() #'spherical_2813220026700000_f.jpg'
            if '_' in imgID:
                imgID = imgID.split('_')[1] #2813220026700000
            else:
                imgID = imgID.split('.')[0]
            ID.append(imgID)
        return ID


    def ID2LL(self,imgID):
        lat,lon,degree2north = -1,-1,-1

        if type(imgID) is not str:
            imgID = imgID[0]

        with open('data_vps/netvlad_etri_datasets/poses.txt', 'r') as searchfile:
            for line in searchfile:
                if imgID in line:
                    sline = line.split('\n')[0].split(' ')
                    lat = sline[1]
                    lon = sline[2]
                    degree2north = sline[3]
        
        return lat,lon,degree2north

    def get_Img_pairs(self):
        for idx in range(len(self.pred_idx)):
            qImg  = Image.open(self.qImage[idx].strip())
            dbImg = Image.open(self.dbImage[self.pred_idx[idx]].strip())
            yield self.ToTensor(qImg),self.ToTensor(dbImg)

    def get_qImgs(self):
        qImgs = []
        for idx in range(self.pred_idx.shape[0]):
            qImg = Image.open(self.qImage[idx].strip())
            qImg = self.ToTensor(qImg.resize((640,480)))
            qImgs.append(qImg)
        return torch.stack(qImgs)

    def get_dbImgs(self):
        dbImgs = []
        for idx in range(self.pred_idx.shape[0]):
            dbImg = Image.open(self.dbImage[self.pred_idx[idx].item(0)].strip())
            dbImg = self.ToTensor(dbImg.resize((640,480)))
            dbImgs.append(dbImg)
        return torch.stack(dbImgs)


if __name__ == "__main__":
    mod_vps = vps()
    mod_vps.initialize()
    qimage = np.uint8(256*np.random.rand(1024,1024,3))
    #(image=None, K=3, gps_lat=None, gps_long=None, gps_accuracy=None, timestamp=None):
    vps_IDandConf = mod_vps.apply(qimage, 3, 37, 27, 95, 100) # k=5 for knn
    print('############## Result of vps().apply():')
    print('vps_IDandConf',vps_IDandConf)

    ## Display Result
    viz = Visdom()
    qImgs  = mod_vps.get_qImgs() #  [10,3,480,640] 
    dbImgs = mod_vps.get_dbImgs() #  [10,3,480,640] 
    qdbImgs = torch.cat((qImgs,dbImgs),-1) #  [10,3,480,1280] 
    img_window = viz.images(qdbImgs,nrow=1,win='Query(left)_DB(right)')

    ## Debugging
    #textwindow = viz.text("[VPS] Results")
    #img_window = viz.images(qImgs,nrow=1,win='Query',opts=dict(title="Query Iamge",caption="Query(ETRI Cart)"))
    #img_window = viz.images(dbImgs,nrow=1,win='DB',opts=dict(title="DB Iamge",caption="DB(Naver)"))
    #vps_lat,vps_long,_,_,_ = mod_vps.apply(qimage)
    #print('Lat,Long =',vps_lat,vps_long)
