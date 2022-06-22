import torch
import torchvision.transforms as transforms
import torch.utils.data as data

import os
from os.path import join, exists
from scipy.io import loadmat
import numpy as np
from random import randint, random
from collections import namedtuple
from PIL import Image
import utm

import matplotlib
matplotlib.use('Agg')  # https://stackoverflow.com/questions/27147300/matplotlib-tcl-asyncdelete-async-handler-deleted-by-the-wrong-thread/29172195#29172195
import matplotlib.pyplot as plt

from sklearn.neighbors import NearestNeighbors
import h5py

import faiss
import cv2

from tictoc import tic, toc
from ipdb import set_trace as bp

named_dbStruct = namedtuple('dbStruct', ['whichSet', 'dataset', 
    'dbImage', 'utmDb', 'qImage', 'utmQ', 'numDb', 'numQ',
    'posDistThr', 'posDistSqThr', 'nonTrivPosDistSqThr'])

class WholeDatasetFromStruct():
    def __init__(self):
        self.abs_image_path = ''
        self.curr_path = os.getcwd()
        self.positives = None
        self.distances = None

    def initialize(self, structFile, db_dir, queries_dir):
        if os.path.exists(structFile) == False:
            print("[vps] ===> Not Found custom dataset : {}".format(structFile))
            return False
        try:
            if 'dg' in structFile.split('/')[-1]:
                self.dbStruct = self.Load_dbStruct(structFile)
            else:
                self.dbStruct = self.parse_dbStruct(structFile)
        except:
            return False
        self.db_dir = db_dir
        self.queries_dir = queries_dir
        ## Only use db image
        self.images = [join(db_dir, dbIm) for dbIm in self.dbStruct.dbImage]
        if len(self.images) == 0:
            return False
        self.whichSet = self.dbStruct.whichSet
        self.dataset = self.dbStruct.dataset
        try:
            self.init_tmp_dir()
        except:
            return False
        self.abs_image_path = os.path.join(self.curr_path, self.db_dir, os.path.dirname(self.dbStruct.dbImage[0]))
        return True

    def get_abs_image_path(self):
        return self.abs_image_path

    def __getitem__(self, index):
        img = get_single_image(self.images[index])

        return img, index

    def __len__(self):
        return len(self.images)

    def getPositiveDBs(self, x=354559.16244696, y=4028082.80969047, coord='utm', radius=None):
        ''' (x, y) = (utm_x, utm_y) 
               or (lat, lon)
        '''
        if coord.lower() == 'latlon':
            x, y, zone_hori, zone_vert = utm.from_latlon(x,y)  # utm.to_latlon(354559.16244696, 4028082.80969047, 52, 'S')

        utm_xy = np.asarray([x,y])
        utm_xy = utm_xy.reshape(-1,2)

        knn = NearestNeighbors(n_jobs=-1)
        #knn = NearestNeighbors(n_jobs=1)
        knn.fit(self.dbStruct.utmDb)
        if radius is None:
            radius = self.dbStruct.posDistThr
        self.distances, self.positives = knn.radius_neighbors(utm_xy, radius=radius)
        #self.distances, self.positives = knn.radius_neighbors(self.dbStruct.utmQ, radius=radius)

        if len(self.positives.item()) > 0:
            return self.positives.item()
        else:
            return None

    def get_dbImage(self):
        return self.dbStruct.dbImage

    def get_qImage(self):
        return self.dbStruct.qImage
    
    def get_utmQ(self):
        return self.dbStruct.utmQ

    def get_utmDb(self):
        return self.dbStruct.utmDb

    def q_idx2utm(self, idx):
        if type(idx) == type(None):
            return None
        try:
            xy = [self.dbStruct.utmQ[i] for i in idx]
        except:  # single value
            xy = self.dbStruct.utmQ[idx]
        return xy

    def q_idx2fname(self, idx):
        if type(idx) == type(None):
            return None
        try:
            fname = [self.dbStruct.qImage[i] for i in idx]
        except:  # single value
            fname = self.dbStruct.qImage[idx]
        return fname

    def db_name2idx(self, name):  # ex) of name is 54800
        dbImage = self.get_dbImage()
        name = "{}.jpg".format(name)
        idx =  [i for i in range(len(dbImage)) if name in dbImage[i]] 
        return idx

    def db_name2utm(self, name):
        utmDb = self.get_utmDb()
        idx = self.db_name2idx(name)
        ret = utmDb[idx]  # array([array([[328528.83696172]]), array([[4153686.71236218]])]
        try:
            x = ret[0][0][0][0]
            y = ret[0][1][0][0]
        except:
            x = ret[0]
            y = ret[1]
        return [x,y]

    def db_idx2utm(self, idx):
        if type(idx) == type(None):
            return None
        try:
            xy = [self.dbStruct.utmDb[i] for i in idx]
        except:  # single value
            xy = self.dbStruct.utmDb[idx]
        return xy

    def db_idx2fname(self, idx):
        if type(idx) == type(None):
            return None
        try:
            fname = [self.dbStruct.dbImage[i] for i in idx]
        except:  # single value
            fname = self.dbStruct.dbImage[idx]
        return fname
    
    def db_idx2FullPath(self, idx):
        if type(idx) == type(None):
            return None
        try:
            fname = [os.path.join(self.db_dir, self.dbStruct.dbImage[i]) for i in idx]
        except:  # single value
            fname = self.dbStruct.dbImage[idx]
        return fname

    def db_getSingleFullPath(self, fname):
        if type(fname) == type(None):
            return None
        FullPath = os.path.join(self.db_dir, fname)
        return FullPath

    def q_getFullPath(self, fname):
        if type(fname) == type(None):
            return None
        FullPath = os.path.join(self.q_dir, fname)
        return FullPath

    def flush_db_dir(self, flush_file="*.jpg"):
        os.system("rm -rf " + os.path.join(self.tmp_db_dir, flush_file)) # You have to pay attention to code 'rm -rf' command

    def GetStreetView(self, x=354559.16244696, y=4028082.80969047, coord='utm', radius=None, outdir=None):
        db_idx = self.getPositiveDBs(x, y, coord=coord, radius=radius)
        if type(db_idx) == type(None):
            print("[vps_custom_image_server] - No db to download near.")
            return -1
        numImgs = len(db_idx)
        if outdir is None:
            outdir = self.tmp_db_dir

        if numImgs > 0:
            ## Flush directory befor downloading jpg images.
            self.flush_db_dir()
            db_paths = self.db_idx2FullPath(db_idx)
            srcs = ""
            link_cnt = 0
            max_single_link_cnt = 200  # max num of ln -sf files : 548
            for f in db_paths:
                '''
                    cp ./netvlad_v100_datasets_dg/././qImg/201007_seoul/000125.jpg /mnt/ramdisk/.vps_dataset/dbImg/StreetView/
                '''
                srcs = "{} {}".format(srcs, os.path.join(self.curr_path, f))
                link_cnt += 1
                if link_cnt >= max_single_link_cnt:
                    os.system("ln -sf {} {}/.".format(srcs, outdir))
                    link_cnt = 0
                    srcs = ""
            if len(srcs) > 0 :
                os.system("ln -sf {} {}/.".format(srcs, outdir))
            return 0
        else:
            return -1

    @staticmethod
    def makedir(fdir):
        if not os.path.exists(fdir):
            os.makedirs(fdir)

    def init_tmp_dir(self):
        self.tmp_rootdir = "/mnt/ramdisk/.vps_dataset"  # tmpdir is symbolic-linked to netvlad_etri_datasets
        self.tmp_q_dir = os.path.join(self.tmp_rootdir, "qImg/999_newquery")
        self.tmp_db_dir = os.path.join(self.tmp_rootdir, "dbImg/StreetView")
        self.makedir(self.tmp_rootdir)
        self.makedir(self.tmp_q_dir)
        self.makedir(self.tmp_db_dir)

    @staticmethod
    def parse_dbStruct(path):
        mat = loadmat(path)
        matStruct = mat['dbStruct'].item()
    
        if '250k' in path.split('/')[-1]:
            dataset = 'pitts250k'
        else:
            dataset = 'pitts30k'
    
        whichSet = matStruct[0].item()
    
        dbImage = [f[0].item() for f in matStruct[1]]
        utmDb = matStruct[2].T
    
        qImage = [f[0].item() for f in matStruct[3]]
        utmQ = matStruct[4].T
    
        numDb = matStruct[5].item()
        numQ = matStruct[6].item()
    
        posDistThr = matStruct[7].item()
        posDistSqThr = matStruct[8].item()
        nonTrivPosDistSqThr = matStruct[9].item()
    
        return named_dbStruct(whichSet, dataset, dbImage, utmDb, qImage, 
                utmQ, numDb, numQ, posDistThr, 
                posDistSqThr, nonTrivPosDistSqThr)
    
    @staticmethod
    def Load_dbStruct(fname):
        mat = loadmat(fname)
        matStruct = mat['dbStruct'][0]
    
        whichSet = matStruct[0].item()
        dataset = matStruct[1].item()
    
        dbImage = [f.item().strip() for f in matStruct[2]]
        utmDb = matStruct[3]
        
        qImage = [f.item().strip() for f in matStruct[4]]
        utmQ = matStruct[5]
        
        numDb = matStruct[6].item()
        numQ = matStruct[7].item()
    
        posDistThr = matStruct[8].item()
        posDistSqThr = matStruct[9].item()
        nonTrivPosDistSqThr = matStruct[10].item()
    
        return named_dbStruct(whichSet, dataset, dbImage, utmDb, qImage,
                utmQ, numDb, numQ, posDistThr,
                posDistSqThr, nonTrivPosDistSqThr)
    
    def get_single_image(self, path):
        #img = Image.open(path)
        #if self.input_transform:
        #    img = self.input_transform(img)
        #img = img.resize((1280, 720))
        img = cv2.imread(path)
        return

    @staticmethod
    def faiss_knn(self, Db, Q, K): 
        Index = faiss.IndexFlatL2(Db.shape[-1])
        Index.add(Db)
        distance, idx = Index.search(Q.reshape(1,-1), K)
        l2norm = np.sqrt(distance).squeeze()
        return l2norm, idx 
    
    @staticmethod
    def input_transform():
        return transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                   std=[0.229, 0.224, 0.225]),
        ])
    
def test_custom_image_server(config):
    custom_dataset = WholeDatasetFromStruct()
    is_custom_dataset_valid = custom_dataset.initialize(config.structFile, config.db_dir, config.queries_dir)
    if is_custom_dataset_valid == False:
        return False

    qImage = custom_dataset.get_qImage()
    for i in range(len(qImage)):
        ## get query
        utmQ = custom_dataset.q_idx2utm(i)

        if type(utmQ) == type(None):
            print("[{}] - No utmQ ".format(i))
            continue

        ## get DB from query's pos.
        db_idx = custom_dataset.getPositiveDBs(utmQ[0], utmQ[1], 'utm', 25)

        if type(db_idx) == type(None):
            print("[{}] - No matched db".format(i))
            continue

        db_len = len(db_idx)
        db_idx = db_idx[0]
        utmDb = custom_dataset.db_idx2utm(i)
        db_fname = custom_dataset.db_idx2fname(db_idx)
        img_path = custom_dataset.db_getSingleFullPath(db_fname)
        print("[{0:06d}] q(x,y): {1:} , db(x,y): {2:}, db_path: {3}".format(i, utmQ, utmDb, img_path))
        #print("{} - db {} images".format(i, db_len))
        db_img = cv2.imread(img_path)
        cv2.imshow("db", db_img)
        cv2.waitKey(1)

        ## Download custom roadview.
        custom_dataset.GetStreetView(utmQ[0], utmQ[1], 'utm', 25)

def test_simple(config):
    custom_dataset = WholeDatasetFromStruct()
    is_custom_dataset_valid = custom_dataset.initialize(config.structFile, config.db_dir, config.queries_dir)
    if is_custom_dataset_valid == False:
        return False

    x, y, coord, radius = config.x, config.y, config.coord, config.radius

    ## Get db information for single position
    db_idx = custom_dataset.getPositiveDBs(x, y, coord, radius)
    if type(db_idx) == type(None):
        print("No matched db")
        return False
    db_fname = custom_dataset.db_idx2fname(db_idx)
    img_paths = custom_dataset.db_idx2FullPath(db_idx)
    img_path = custom_dataset.db_getSingleFullPath(db_fname[0])
    utmDb = custom_dataset.db_idx2utm(db_idx)

    print(img_paths)

    return True

def test_download(config):
    custom_dataset = WholeDatasetFromStruct()
    is_custom_dataset_valid = custom_dataset.initialize(config.structFile, config.db_dir, config.queries_dir)
    if is_custom_dataset_valid == False:
        return False
    x, y, coord, radius = config.x, config.y, config.coord, config.radius

    ## Download custom roadview.
    custom_dataset.GetStreetView(x, y, coord, radius)

    return True

if __name__ == "__main__":
    if True:  # Seoul
        import config_seoul as config

    if False:  # Daejeon
        import config_daejeon as config

    #test_simple(config)
    test_download(config)
    #test_custom_image_server(config)
