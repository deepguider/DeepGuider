from __future__ import print_function
import argparse
from math import log10, ceil
import random
import shutil
import json
from os.path import join, exists, isfile, realpath, dirname
from os import makedirs, remove, chdir, environ
import os
import glob # may cause segmentation fault in (C+Python) environment

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
#import h5py
import faiss
import time

import utm

#from tensorboardX import SummaryWriter
import numpy as np
from netvlad import netvlad

from scipy import io as sio

import copy

import cv2 as cv

from get_streetview import ImgServer, GetStreetView_fromID

from netvlad import etri_dbloader as dataset

from vps_filter import vps_filter

from ipdb import set_trace as bp

class vps:
    def __init__(self, which_gpu=0, region="ETRI"):
        self.ipaddr = 'localhost'
        self.gps_lat = 0.0 #Latitude
        self.gps_lon = 0.0 #Longitude
        self.vps_lat = 0.0 # Latitude from VPS function
        self.vps_long = 0.0 # Longitude from VPS function
        self.angle = -1  # road direction (radian)
        self.vps_prob = -1   # reliablity of the result. 0: fail ~ 1: success
        self.K = int(3) # K for Top-K for best matching
        if self.init_vps_IDandConf(self.K) < 0: #init_vps_IDandConf after setting self.K
            bp()
        self.ToTensor = transforms.ToTensor()
        self.verbose = False # 1 : print internal results
        self.StreetViewServerAvaiable = True
        self.callcounter_gSV = 0 # N of call of getStreetView(), for debugging purpose
        device = 'cuda:{}'.format(which_gpu) if torch.cuda.is_available() else 'cpu'  #cuda:0
        self.device = torch.device(device)
        self.set_region(region)
        self.load_dbfeat_initialized = False

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
        self.parser.add_argument('--seed', type=int, default=123, help='Random seed to use.')
        self.parser.add_argument('--dataPath', type=str, default='netlvad/netvlad_v100_datasets/', help='Path for centroid data.')
        self.parser.add_argument('--runsPath', type=str, default='netvlad/checkpoints/runs/', help='Path to save runs to.')
        self.parser.add_argument('--savePath', type=str, default='checkpoints', 
                help='Path to save checkpoints to in logdir. Default=netvlad/checkpoints/')
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
        self.parser.add_argument('--dataset', type=str, default='deepguider', help='Dataset to use', choices=['pittsburgh','deepguider'])
        self.parser.add_argument('--cacheBatchSize', type=int, default=1, help='Batch size for caching and testing')

        self.parser.add_argument('--dbFeat_fname', type=str, default='data_vps/prebuilt_dbFeat.mat', help='dbFeat file calculated in advance')
        self.parser.add_argument('--qFeat_fname', type=str, default='data_vps/prebuilt_qFeat.mat', help='dbFeat file calculated in advance')
        self.parser.add_argument('--verbose', default=False, action='store_true', help='Print internal messages') #fixed, dg's issue #41
        
        # When you get 'ERROR: Unexpected segmentation fault encountered in worker'
        # then, set threads to 0
        self.parser.add_argument('--threads', type=int, default=0, help='Number of threads for each data loader to use') #fixed, dg'issue #42
        # self.parser.add_argument('--threads', type=int, default=8, help='Number of threads for each data loader to use')

        self.parser.add_argument('--ipaddr', type=str, default='127.0.0.1', help='ip address of streetview server')
        self.parser.add_argument('--port', type=str, default='10000', help='port of streetview server, 10000:ETRI, 10001:COEX, 10002:Bucheon, 10003:ETRI Indoor')
        ######(end) Following defaults are combination of 9run_vps_ccsmm.sh

        return 1 # It has to return positive value to C++

    def initialize(self):
        self.init_param()
        opt = self.parser.parse_args()
        self.verbose = opt.verbose
        self.ipaddr = opt.ipaddr
        self.port = opt.port
        self.set_cubic_str('f')  ## Default is 'f'
        self.PythonOnly = True # This is parameter should become False when vps is used in embedded module by C++ to avoid segmentation fault.

        self.num_workers = opt.threads
        self.cacheBatchSize = opt.cacheBatchSize

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
                    if self.verbose:
                        print('Restored flags:', train_flags)
                    opt = self.parser.parse_args(train_flags, namespace=opt)
    
        if self.verbose:
            print(opt)
    
        cuda = not opt.nocuda
        if cuda and not torch.cuda.is_available():
            raise Exception("No GPU found, please run with --nocuda")
    
        random.seed(opt.seed)
        np.random.seed(opt.seed)
        torch.manual_seed(opt.seed)
        if cuda:
            torch.cuda.manual_seed(opt.seed)
      
        if self.verbose:
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
                if False and (not opt.resume):   # Do not use this in test/apply mode due to version mismatching between h5py and HDF5.
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
            model = model.to(self.device)
        
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
                    p=2, reduction='sum').to(self.device)
    
        if opt.resume:
            if opt.ckpt.lower() == 'latest':
                resume_ckpt = join(opt.resume, 'checkpoints', 'checkpoint.pth.tar')
            elif opt.ckpt.lower() == 'best':
                resume_ckpt = join(opt.resume, 'checkpoints', 'model_best.pth.tar')
    
            if isfile(resume_ckpt):
                if self.verbose:
                    print("=> loading checkpoint '{}'".format(resume_ckpt))
                checkpoint = torch.load(resume_ckpt, map_location=lambda storage, loc: storage)
                opt.start_epoch = checkpoint['epoch']
                best_metric = checkpoint['best_score']
                model.load_state_dict(checkpoint['state_dict'])
                model = model.to(self.device)
                if opt.mode == 'train':
                    optimizer.load_state_dict(checkpoint['optimizer'])
                if self.verbose:
                    print("=> loaded checkpoint '{}' (epoch {})"
                          .format(resume_ckpt, checkpoint['epoch']))
            else:
                print("=> no checkpoint found at '{}'".format(resume_ckpt))
    
        self.model = model
        if self.verbose:
            print('===> Building model end(vps.py)')

        self.mVps_filter = vps_filter()

        if opt.dataset.lower() == 'pittsburgh':
            from netvlad import pittsburgh as dataset
            return 0 # Failed

        elif opt.dataset.lower() == 'deepguider':
            from netvlad import etri_dbloader as dataset
            import os
            self.dataset_root_dir = dataset.root_dir
            self.dataset_struct_dir = os.path.join(dataset.struct_dir,'StreetView')
            self.dataset_queries_dir = os.path.join(dataset.queries_dir,'999_newquery')
            self.makedir(self.dataset_struct_dir)
            self.makedir(self.dataset_queries_dir)
            return 1 # Non-zero means success return 
    
    def set_threads(self, num_workers=0):
        if num_workers > 0 :
            self.num_workers = num_workers

    def set_cacheBatchSize(self, cacheBatchSize=1):
        if cacheBatchSize > 1:
            self.cacheBatchSize = cacheBatchSize 

    def test_sub(self,eval_set,epoch=0):
        opt = self.parser.parse_args()
        cuda = not opt.nocuda
        test_data_loader = DataLoader(dataset=eval_set, 
                    num_workers=self.num_workers, batch_size=self.cacheBatchSize, shuffle=False, 
                    pin_memory=cuda)
                    # num_workers = 0 means that the data will be loaded in the main process. (default: 0)
        self.model.eval()
        with torch.no_grad():
            if self.verbose:
                print('====> Extracting Features')
            pool_size = self.encoder_dim
            if opt.pooling.lower() == 'netvlad': pool_size *= opt.num_clusters
            Feat = np.zeros([len(eval_set), pool_size])
            for iteration, (input, indices) in enumerate(test_data_loader, 1):
                #if input == None: # for broken input data, set output to zero
                if len(input.shape) == 1: # for broken input data, set output to zero
                    bp()
                    continue
                input = input.to(self.device) #[24, 3, 480, 640]
                try:
                    image_encoding = self.model.encoder(input) #[24, 512, 30, 40]
                except: # for broken input data, set output to zero
                    continue
                vlad_encoding = self.model.pool(image_encoding) #[24,32768] 
                #Feat : [17608, 32768]
                Feat[indices.detach().numpy(), :] = copy.deepcopy(vlad_encoding.detach().cpu().numpy()) #[24,32768]
                if self.verbose:
                    try:
                        if (iteration-1) % 200 == 0:
                            print("==> Batch ({}/{})".format(iteration,len(test_data_loader)), flush=True)
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
    
    def make_search_index(self, dbFeat):
        num_db, pool_size = dbFeat.shape # NumDB,32768
        self.faiss_index = faiss.IndexFlatL2(pool_size) # fixed, dg's issue #21. It uses similarity(confidence) as metric 
        self.faiss_index.add(dbFeat)
        return self.faiss_index

    def test_dg(self,eval_set_db,eval_set_q, epoch=0, write_tboard=False):
        # TODO what if features dont fit in memory? 
        opt = self.parser.parse_args()
        cuda = not opt.nocuda

        if self.load_dbfeat == True:
            if self.load_dbfeat_initialized == False: ## Initial condition is False (off)
                print('[vps]====> Initializing load_dbfeat.')
                if self.verbose:
                    print('====> Initializing load_dbfeat.')
                dbFeat_dict = sio.loadmat(opt.dbFeat_fname)
                dbFeat = dbFeat_dict['Feat']
                self.dbImage = dbFeat_dict['dbImage']
                dbFeat = np.ascontiguousarray(dbFeat)
                self.faiss_index = self.make_search_index(dbFeat)
                self.load_dbfeat_initialized = True  ## Turn on the switch after loading dbfeat.
            ## Reuse faiss_index made in previous call
            faiss_index = self.faiss_index
            dbImage = self.dbImage
        else:  # Calculate DB features everytime
            if (eval_set_db == None) or (eval_set_q == None):
                return -1
            else:
                if len(eval_set_db.images) < self.K:
                    return -1
                if len(eval_set_db.images) < 1:
                    return -1
                if len(eval_set_q.images) < 1:
                    return -1
            ## extracted for db, now split in own sets
            #print('[vps]====> Extracting dbfeat.')
            if self.verbose:
                print('====> Extracting dbfeat.')
            dbFeat = self.test_sub(eval_set_db, epoch=epoch)
            dbFeat = dbFeat.astype('float32') #[ndbImg,32768]
            test_db_data_loader = DataLoader(dataset=eval_set_db, 
                num_workers=self.num_workers, batch_size=self.cacheBatchSize, shuffle=False, 
                pin_memory=cuda)
            dbImage = test_db_data_loader.dataset.ImgStruct.Image
            faiss_index = self.make_search_index(dbFeat)
            if self.save_dbfeat:
                if self.verbose:
                    print('====> Saving dbfeat into ', opt.dbFeat_fname)
                dbFeat_dict={'Feat':dbFeat, 'dbImage':dbImage}
                sio.savemat(opt.dbFeat_fname, dbFeat_dict) # savemat may cause segmentation fault randomly when embedded in C++.

        if self.verbose:
            print('====> Building faiss index')

        # extracted for query, now split in own sets
        qFeat = self.test_sub(eval_set_q,epoch=epoch)
        qFeat = qFeat.astype('float32') #[nqImg,32768]

        test_q_data_loader = DataLoader(dataset=eval_set_q, 
                num_workers=self.num_workers, batch_size=self.cacheBatchSize, shuffle=False, 
                pin_memory=cuda)

        if self.verbose:
            print('====> Calculating recall @',self.K)
        
        pred_confidence, pred_idx = faiss_index.search(qFeat, self.K) # [ NumQ x K ]
  
        if self.verbose:
            print('predicted ID:\n', pred_idx)
  
        qImage  = test_q_data_loader.dataset.ImgStruct.Image
  
        self.qImage = qImage
        self.dbImage = dbImage
        self.pred_idx = pred_idx
        self.pred_confidence = pred_confidence

        if self.verbose:
            print('QueryImage <=================> predicted dbImage')
        match_cnt = 0
        total_cnt = len(qImage)

        if total_cnt == 0:
            return 0
        for i in range(total_cnt):
            qName = os.path.basename(qImage[i].item()).strip()
            if qName in 'newquery.jpg':
                flist = dbImage[pred_idx[i]]
                vps_imgID_str = self.Fname2ID(flist) # list
                vps_imgConf_str = [val for val in pred_confidence[i]] # list
                vps_imgID = [np.int(ii) for ii in vps_imgID_str] # fixed, dg'issue #36
                vps_imgConf = [np.float(ii) for ii in vps_imgConf_str] # fixed, dg'issue #36
                self.vps_IDandConf = [vps_imgID, vps_imgConf]
 
            if self.verbose:
                dbImage_predicted = dbImage[pred_idx[i,0]] #Use best [0] image for display
                try:
                    dbName_predicted = os.path.basename(dbImage_predicted[i].item()).strip()
                except:
                    dbName_predicted = os.path.basename(dbImage_predicted[i].strip())

                #IDs = ['spherical_2812920067800000','spherical_2812920067800000']
                lat,lon,deg = self.ID2LL(self.Fname2ID(dbName_predicted))
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

    def checking_return_value(self):
        K = self.K
        vps_imgID = self.vps_IDandConf[0]
        vps_imgConf = self.vps_IDandConf[1]
        if (len(vps_imgID) != K) or (len(vps_imgConf) != K):
            return -1
        ErrCnt = K
        for i in vps_imgID:
             if (isinstance(vps_imgID[0],int) == False):
                 ErrCnt = ErrCnt - 1
        if K != ErrCnt:
            return -1
        ErrCnt = K
        for i in vps_imgConf:
             if (isinstance(vps_imgConf[0],float) == False):
                 ErrCnt = ErrCnt - 1
        if K != ErrCnt:
            return -1
        return 0


    def test(self,eval_set, epoch=0, write_tboard=False):
        # TODO what if features dont fit in memory? 
        opt = self.parser.parse_args()
        cuda = not opt.nocuda

        test_data_loader = DataLoader(dataset=eval_set, 
                    num_workers=self.num_workers, batch_size=self.cacheBatchSize, shuffle=False, 
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
                input = input.to(self.device) #[24, 3, 480, 640]
                image_encoding = self.model.encoder(input) #[24, 512, 30, 40]
                vlad_encoding = self.model.pool(image_encoding) #[24,32768] 
        
                #dbqFeat : [17608, 32768]
                dbqFeat[indices.detach().numpy(), :] = vlad_encoding.detach().cpu().numpy() #[24,32768]
                try:
                    if iteration % 50 == 0 or len(test_data_loader) <= 10:
                        print("==> Batch ({}/{})".format(iteration,len(test_data_loader)), flush=True)
                except:
                    print("Cannot Display")
            del input, image_encoding, vlad_encoding
        del test_data_loader

        if self.load_dbfeat == True:
            dbFeat = sio.loadmat(opt.dbFeat_fname)
            dbFeat = dbFeat['dbFeat']
        else:
            # extracted for db, now split in own sets
            dbFeat = dbqFeat[:eval_set.dbStruct.numDb].astype('float32') #[10000,32768]
            dbFeat_dict={'dbFeat':dbFeat}
            if self.save_dbfeat:
                sio.savemat(opt.dbFeat_fname,dbFeat_dict)

        # extracted for query, now split in own sets
        qFeat = dbqFeat[eval_set.dbStruct.numDb:].astype('float32') #[7608,32768]
        qFeat_dict={'qFeat':qFeat}

        print('====> Building faiss index')
        #qFeat  : [7608,32768], pool_size = 32768 as dimension of feature
        #dbFeat : [10000,32768]
        faiss_index = faiss.IndexFlatL2(pool_size) #32768
        faiss_index.add(dbFeat)
    
        print('====> Calculating recall @ N')
        n_values = [1,5,10,20] #n nearest neighbors
    
        pred_confidence, pred_idx = faiss_index.search(qFeat, self.K) #predictions : [7608,1]

        print('predicted ID:\n', pred_idx)
        test_data_loader = DataLoader(dataset=eval_set, 
                    num_workers=self.num_workers, batch_size=self.cacheBatchSize, shuffle=False, 
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
                vps_imgConf = [val for val in pred_confidence[i]]
                self.vps_IDandConf = [vps_imgID, vps_imgConf] #ori
                #self.vps_IDandConf = [123456, 1.33] #dbg

            if self.Fname2ID(qName)[0] in self.Fname2ID(dbName_predicted)[0]:
                match_cnt = match_cnt + 1
                print('[Q]',qName,'<==> [Pred]', dbName_predicted,'[Lat,Lon] =',lat,',',lon,'[*Matched]')
            else:
                print('[Q]',qName,'<==> [Pred]', dbName_predicted,'[Lat,Lon] =',lat,',',lon)

        acc = match_cnt/total_cnt
        print('Accuracy : {} / {} = {} % in {} DB images'.format(match_cnt,total_cnt,acc*100.0,len(dbImage)))
        return acc

        
    def get_param(self):
        # get parameter
        return 0
        

    def set_param(self):
        # set parameter
        return 0


    def init_vps_IDandConf(self,K):
        # vps_imgID = [int(i) for i in np.zeros(K)] # list of uint64, fixed dg'issue #36
        # vps_imgConf = [float(i) for i in np.zeros(K)] # list of double(float64), fixed dg'issue #36
        vps_imgID = [int(0) for i in np.zeros(K)] # list of uint64, fixed dg'issue #36
        vps_imgConf = [float(-1) for i in np.zeros(K)] # list of double(float64), fixed dg'issue #36
        self.vps_IDandConf = [vps_imgID, vps_imgConf]
        return 0


    def makedir(self,fdir):
        import os
        if not os.path.exists(fdir):
            os.makedirs(fdir)

    def set_region(self, region="ETRI"):  # region information for image server used in isv.SaveImages
        self.region = region

    def get_region(self):  # region information for image server used in isv.SaveImages
        return self.region

    def flush_db_dir(self):
        os.system("rm -rf " + os.path.join(self.dataset_struct_dir,'*.jpg')) # You have to pay attention to code 'rm -rf' command

    def apply(self, image=None, K = 3, gps_lat=37.0, gps_lon=127.0, gps_accuracy=0.79, timestamp=0.0, ipaddr=None, port=None, load_dbfeat=0.0, save_dbfeat=0.0):
        ## Init.
        if ipaddr != None:
            self.ipaddr = ipaddr
        if port != None:
            self.port = port

        if load_dbfeat > 0:
            self.load_dbfeat = True
        else:
            self.load_dbfeat = False

        if save_dbfeat > 0:
            self.save_dbfeat = True
        else:
            self.save_dbfeat = False

        self.gps_lat = float(gps_lat)
        self.gps_lon = float(gps_lon)
        self.gps_accuracy = min(max(gps_accuracy,0.0),1.0)
        self.timestamp = float(timestamp)
        self.K = int(K);
        self.init_vps_IDandConf(self.K)
        opt = self.parser.parse_args()
        self.set_radius_by_accuracy(self.gps_accuracy)
        #self.set_radius(30)  # meters
        self.set_cubic_str('f')

        ret = -1

        if self.verbose:
            print('===> Loading dataset(s)')
        epoch = 1
        ## Load dataset
        if opt.dataset.lower() == 'pittsburgh':
            from netvlad import pittsburgh as dataset
            whole_test_set = dataset.get_whole_test_set()
            print('===> Evaluating on test set')
            print('===> Running evaluation step')
            recalls = self.test(whole_test_set, epoch, write_tboard=False)
        elif opt.dataset.lower() == 'deepguider':
            from netvlad import etri_dbloader as dataset
            if self.load_dbfeat == False:
                ## Get DB images from streetview image server            
                ret = self.getStreetView(self.dataset_struct_dir)               
                if ret < 0:
                    print("[vps] Local DB and features are used : ", self.dataset_struct_dir)
            else:
                self.flush_db_dir()

            if image is not None:
                fname = os.path.join(self.dataset_queries_dir,'newquery.jpg')
                try:
                    h, w, c = image.shape
                except: # invalid query image
                    self.flush_db_dir()
                    return self.getIDConf()
                if (h < 480) or (w < 640) or (c != 3): # invalid query image
                    self.flush_db_dir()
                    return self.getIDConf()
                cv.imwrite(fname,image)

            if self.port == "10003":  # input image resolution : 2592*2048
                whole_db_set,whole_q_set = dataset.get_dg_indoor_test_set()
            else:  # input image resolution : 1024*1024
                whole_db_set,whole_q_set = dataset.get_dg_test_set()

            if self.verbose:
                print('===> With Query captured near the ETRI Campus')
                print('===> Evaluating on test set')
                print('===> Running evaluation step')
            ## Calculate image feature, vlad feature and do matching of query and DBs
            acc = self.test_dg(whole_db_set,whole_q_set, epoch, write_tboard=False) #may cause segmentation fault.
        else:
            raise Exception('Unknown dataset')

        ## Return [ [id1,id2,...,idN],[conf1,conf2,...,confidenceN]]        
        self.flush_db_dir()
        return self.getIDConf()

    def convert_distance_to_confidence(self, distances, sigma=0.2):  # distances is list type
        confidences = []
        for dist in distances:
            conf = np.exp(-1*sigma*dist)
            confidences.append(conf)
        return confidences

    def getIDConf(self):
        if self.checking_return_value() < 0:
            print("Broken : vps.py's return value")
        IDs = self.vps_IDandConf[0]
        Distances = self.vps_IDandConf[1]
        Confs = self.convert_distance_to_confidence(Distances)

        ## Filter out noisy result with ransac for top-1
        top1_id = IDs[0]
        _, lat, lon = GetStreetView_fromID(top1_id, roi_radius=1, ipaddr=self.ipaddr)
        if lat != -1:  # Image server is ready.
            # When image server is not available, do not filter out.
            utm_x, utm_y, r_num, r_str = utm.from_latlon(lat, lon)  # (353618.4250711136, 4027830.874694569, 52, 'S')
            #utm_x = utm_x - 353618  # offset
            #utm_y = utm_y - 4027830  # offset
            if self.mVps_filter.check_valid(utm_x, utm_y) == False:   # Filter out noisy result with ransac of first-order line function
                # Noisy result is changed to -1.
                IDs[0] = 0
                Confs[0] = -1
        return [IDs, Confs]

    def set_radius_by_accuracy(self, gps_accuracy=0.79):
        self.roi_radius = int(10 + 190*(1-gps_accuracy))  # 10 meters ~ 200 meters, 0.79 for 50 meters
        return 0

    def set_radius(self, roi_radius=50):  # meters
        self.roi_radius = roi_radius
        return 0

    def set_cubic_str(self, cubic_str='f'):
        if cubic_str not in ['f', 'b', 'l', 'r', 'u', 'd']:
            cubic_str =''  # '' means panoramic image
        if self.port == "10003":  ## 10000:ETRI, 10001:COEX, 10002:Bucheon, 10003:ETRI Indoor
            ## for indoor : '0', '1', '2', ''(panoramic)
            if cubic_str == 'f': cubic_str = '1'  # frontal
            if cubic_str == 'l': cubic_str = '0'  # more left
            if cubic_str == 'r': cubic_str = '2'  # left
        self.cubic_str = cubic_str
        return self.cubic_str

    def get_cubic_str(self):
        return self.cubic_str

    def getStreetView(self, outdir='./'):
        server_type = "streetview"
        req_type = "wgs"
        isv = ImgServer(self.ipaddr, self.port)
        isv.SetServerType(server_type)
        isv.SetParamsWGS(self.gps_lat,self.gps_lon,self.roi_radius) # 37,27,100
        isv.SetReqDict(req_type)
        ret = isv.QuerytoServer(json_save=True, outdir=outdir, PythonOnly=self.PythonOnly)
        if ret == -1:
            print('[vps] Image server({}) is not available. Local DB is used.'.format(self.ipaddr))
            return -1
        numImgs = isv.GetNumImgs()
        if numImgs >0: 
            imgID,imgLat,imgLong,imgDate,imgHeading,numImgs = isv.GetStreetViewInfo(0)
            os.system("rm -rf " + os.path.join(outdir,'*.jpg')) # You have to pay attention to code 'rm -rf' command
            ret = isv.SaveImages(outdir=outdir, cubic=self.get_cubic_str(), verbose=0, PythonOnly=self.PythonOnly)  # original
            ## Use 3 direction images for indoor test 
            if False:
                ret = isv.SaveImages(outdir=outdir, cubic=self.set_cubic_str('f'), verbose=0, PythonOnly=self.PythonOnly)  # original
                ret = isv.SaveImages(outdir=outdir, cubic=self.set_cubic_str('l'), verbose=0, PythonOnly=self.PythonOnly)  # original
                ret = isv.SaveImages(outdir=outdir, cubic=self.set_cubic_str('r'), verbose=0, PythonOnly=self.PythonOnly)  # original
            if ret == -1:
                print('[vps] Image server({}) is not available. Local DB is used.'.format(self.ipaddr))
                return -1
        else:
            os.system("rm -rf " + os.path.join(outdir,'*.jpg')) # You have to pay attention to code 'rm -rf' command
        return 0

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
                imgID = imgID.split('_')[-2] #2813220026700000
            else:
                imgID = imgID.split('.')[0]
            ID.append(imgID) # string
        return ID


    def ID2LL(self,imgID):
        lat,lon,degree2north = -1,-1,-1
        if type(imgID) is not str:
            imgID = imgID[0]
        #'data_vps/netvlad_etri_datasets/poses.txt'
        fname = os.path.join(self.dataset_root_dir,'poses.txt')
        with open(fname, 'r') as searchfile:
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


def run_prebuilt_dbfeat(load_dbfeat=0, save_dbfeat=0):
    from netvlad import etri_dbloader as dataset
    from PIL import Image
    #streetview_server_ipaddr = "localhost"
    streetview_server_ipaddr = "extract.feature.local.db"  # Instead of downloading db, local saved image file is used. For doing it, wrong ip address is used.
    ## 10000:ETRI, 10001:COEX, 10002:Bucheon, 10003:Indoor
    streetview_server_port = "10003";gps_lat, gps_lon = 36.380018, 127.368114
    #streetview_server_port = "10000"; gps_lat, gps_lon = 36.3845257,127.3768796

    #qFlist = dataset.Generate_Flist('/home/ccsmm/Naverlabs/query_etri_cart/images_2019_11_15_12_45_11',".jpg")
    qFlist = dataset.Generate_Flist("data_vps/netvlad_etri_datasets/qImg/999_newquery",".jpg")
    print("Initializing vps module...")
    st = time.time()
    mod_vps = vps()
    mod_vps.initialize()
    mod_vps.verbose = True
    print("It took {} sec.".format(time.time()-st))

    if load_dbfeat > 0:
        print("Run VPS for testing [Loading prebuilt dbfeat into data_vps/prebuilt_dbFeat.mat]")

    if save_dbfeat > 0:
        db_image_dir = os.path.join(dataset.struct_dir,'StreetView')
        print("Run VPS for testing [Saving prebuilt dbfeat into data_vps/prebuilt_dbFeat.mat] of {}".format(db_image_dir))
        if True:  # Fast feature extract at offline
            mod_vps.set_threads(16)
            mod_vps.set_cacheBatchSize(256)

    #qimage = np.uint8(256*np.random.rand(1024,1024,3))
    for fname in qFlist:
        qimg = cv.imread(fname)
        try:
            [h, w, c] = qimg.shape
            if (h < 480) or (w < 640) or c != 3:
                print("Invalid shape of query image :",fname,h,w,c)
                continue
        except:
            print("Broken query image :", fname)
            continue
        qimg = cv.resize(qimg, (640,480))
        st = time.time()
        vps_IDandConf = mod_vps.apply(qimg, K=3, gps_lat=gps_lat, gps_lon=gps_lon, gps_accuracy=0.0,
                timestamp=1.0, ipaddr=streetview_server_ipaddr, port=streetview_server_port,
                load_dbfeat=load_dbfeat, save_dbfeat=save_dbfeat) # k=3 for knn
        print('vps_IDandConf : {}\n{} sec. elapsed.'.format(vps_IDandConf, time.time() - st))

def save_prebuild_dbfeat():
    run_prebuilt_dbfeat(load_dbfeat=0, save_dbfeat=1)

def load_prebuild_dbfeat():
    run_prebuilt_dbfeat(load_dbfeat=1, save_dbfeat=0)

if __name__ == "__main__":
    #save_prebuild_dbfeat()
    load_prebuild_dbfeat()
