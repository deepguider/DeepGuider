import torch
import torchvision.transforms as transforms
import torch.utils.data as data

from os.path import join, exists
from scipy.io import loadmat
import numpy as np
from random import randint, random
from collections import namedtuple
from PIL import Image

from sklearn.neighbors import NearestNeighbors

from ipdb import set_trace as bp

import tempfile

def makedir(fdir):
    import os
    if not os.path.exists(fdir):
        os.makedirs(fdir)

#root_dir = './netvlad_v100_datasets/' #you need this directory in the top.
#root_dir = './data_vps/netvlad_etri_datasets/' #you need this directory in the top.
root_top = join('.','data_vps','netvlad_etri_datasets')

if not exists(root_top):
    msg = 'Not found [{}] for saving street view images. '.format(root_top) + \
    'Please adjust root_top at etri_dbloader.py'
    raise FileNotFoundError(msg)


USE_TEMP_NAME = False
if USE_TEMP_NAME:
    while True:
        temp_name = '.' + next(tempfile._get_candidate_names())
        root_dir = join(root_top, temp_name)
        if not exists(root_dir):
            break
else:
    root_dir = root_top

makedir(root_dir)

struct_dir = join(root_dir, 'dbImg')
queries_dir = join(root_dir, 'qImg')

from scipy import io as sio
import numpy as np
import os,glob
from collections import namedtuple


def Generate_Flist(rawdir,ftype='*.jpg'):
    #ftype = '*.jpg'
    files =[]
    for dirname, dirnames, filenames in os.walk(rawdir):
        for filename in filenames:
            _, ext = os.path.splitext(filename)
            if ext in ftype:
                if 0: #filename with only parents folder
                    fname_with_parents = os.path.join(os.path.basename(dirname), os.path.basename(filename))
                else: #full name 
                    fname_with_parents = os.path.join(dirname,filename)
                fname_with_parents = fname_with_parents.strip() #remove white space in string
                files.append(fname_with_parents)
#    files = np.array(files)
    return files


def GetFlistDict(db_dir,queries_dir):
    dbFlist = Generate_Flist(db_dir)
    qFlist  = Generate_Flist(queries_dir)
    dbFlist_dict = {'Flist':dbFlist}
    qFlist_dict  = {'Flist':qFlist}
    return dbFlist_dict, qFlist_dict


def SaveMatFiles(db_dir,queries_dir,dbMat_fname,qMat_fname):
    dbFlist_dict, qFlist_dict = GetFlistDict(db_dir,queries_dir)
    sio.savemat(dbMat_fname,dbFlist_dict)
    sio.savemat(qMat_fname,qFlist_dict)


def LoadMatFile(Mat_fname):
    Mat = sio.loadmat(Mat_fname)

    dataset = 'etridb'
    whichSet = 'test'

    Images = Mat['Flist']

    numImgs = len(Images)

    utm = np.random.rand(2,numImgs)

    posDistThr = 25
    posDistSqThr = 625
    nonTrivPosDistSqThr = 100

    return ImgStruct(whichSet, dataset, Images, utm, numImgs, posDistThr, 
            posDistSqThr, nonTrivPosDistSqThr)


def LoadDict(Flist_dict):
    Mat = Flist_dict

    dataset = 'etridb'
    whichSet = 'test'

    Images = np.array(Mat['Flist'])

    numImgs = len(Images)

    utm = np.random.rand(2,numImgs)

    posDistThr = 25
    posDistSqThr = 625
    nonTrivPosDistSqThr = 100

    return ImgStruct(whichSet, dataset, Images, utm, numImgs, posDistThr, 
            posDistSqThr, nonTrivPosDistSqThr)


def LoadMatFiles(dbMat_fname,qMat_fname):
    dbMat = sio.loadmat(dbMat_fname)
    qMat = sio.loadmat(qMat_fname)

    dataset = 'etridb'
    whichSet = 'test'

    dbImage = dbMat['Flist']
    qImage = qMat['Flist']

    numDb = len(dbImage)
    numQ = len(qImage)

    utmDb = np.random.rand(2,numDb)
    utmQ = np.random.rand(2,numQ)

    posDistThr = 25
    posDistSqThr = 625
    nonTrivPosDistSqThr = 100

    return dbStruct(whichSet, dataset, dbImage, utmDb, qImage, 
            utmQ, numDb, numQ, posDistThr, 
            posDistSqThr, nonTrivPosDistSqThr)


def input_transform_q():  # Input shape is 640*480
    return transforms.Compose([
        transforms.Resize((240,320)),  # H,W
        transforms.CenterCrop((240,320)),  # H,W
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225]),
    ])

def input_transform_db():  # Input shape changes, 1024*1024 for streetview,
    return transforms.Compose([
        transforms.CenterCrop((768,1024)), ## H, W, Crop car bonnet (hood)
        transforms.Resize((240,320)),  # H, W 
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225]),
    ])

def input_transform_db_indoor():  # Input shape changes, 2592*2048 for indoor streetview
    return transforms.Compose([
        transforms.Resize((240,320)),  # H, W 
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                               std=[0.229, 0.224, 0.225]),
    ])

#input_transform = input_transform_db()


def get_dg_test_set_using_matfile(dbDir='dbImg',qDir='qImg'):
    ## Usage : whole_db_set,whole_q_set = dataset.get_dg_test_set_using_matfile()
    #datasetDir = './netvlad_etri_datasets'
    datasetDir = root_dir
    db_dir = os.path.join(datasetDir, dbDir)
    queries_dir = os.path.join(datasetDir, qDir)
    dbMat_fname = join(root_dir,'etri_db.mat')
    qMat_fname = join(root_dir,'etri_query.mat')
    SaveMatFiles(db_dir,queries_dir,dbMat_fname,qMat_fname) #Run this when db changes
    dbFlist_dict, qFlist_dict = GetFlistDict(db_dir,queries_dir)
    return DG_DatasetFromStruct(dbMat_fname, dbFlist_dict, input_transform=input_transform_db()),\
            DG_DatasetFromStruct(qMat_fname, qFlist_dict, input_transform=input_transform_q())

def get_dg_indoor_test_set(dbDir='dbImg',qDir='qImg'):
    datasetDir = root_dir
    db_dir = os.path.join(datasetDir, dbDir)
    queries_dir = os.path.join(datasetDir, qDir)
    dbFlist_dict, qFlist_dict = GetFlistDict(db_dir,queries_dir)
    return DG_DatasetFromStruct(None, dbFlist_dict, input_transform=input_transform_db_indoor()),\
            DG_DatasetFromStruct(None, qFlist_dict, input_transform=input_transform_q())

def get_dg_test_set(dbDir='dbImg',qDir='qImg'):
    datasetDir = root_dir
    db_dir = os.path.join(datasetDir, dbDir)
    queries_dir = os.path.join(datasetDir, qDir)
    dbFlist_dict, qFlist_dict = GetFlistDict(db_dir,queries_dir)
    return DG_DatasetFromStruct(None, dbFlist_dict, input_transform=input_transform_db()),\
            DG_DatasetFromStruct(None, qFlist_dict, input_transform=input_transform_q())

dbStruct = namedtuple('dbStruct', ['whichSet', 'dataset', 
    'dbImage', 'utmDb', 'qImage', 'utmQ', 'numDb', 'numQ',
    'posDistThr', 'posDistSqThr', 'nonTrivPosDistSqThr'])

ImgStruct = namedtuple('ImgStruct', ['whichSet', 'dataset', 
    'Image', 'utm', 'numImg', 'posDistThr', 'posDistSqThr', 'nonTrivPosDistSqThr'])


class DG_DatasetFromStruct(data.Dataset):
    def __init__(self, Mat_fname, Flist_dict,  input_transform=None, onlyDB=False):
        super().__init__()

        self.input_transform = input_transform
        if Mat_fname == None:
            self.ImgStruct = LoadDict(Flist_dict)
        else:
            self.ImgStruct = LoadMatFile(Mat_fname)

        self.images = [img.strip() for img in self.ImgStruct.Image]
        self.whichSet = self.ImgStruct.whichSet
        self.dataset = self.ImgStruct.dataset
        self.positives = None
        self.distances = None
        self.posDistThr = self.ImgStruct.posDistThr
        self.posDistSqThr = self.ImgStruct.posDistSqThr
        self.nonTrivPosDistSqThr = self.ImgStruct.nonTrivPosDistSqThr

    def __getitem__(self, index):
        try:
            img = Image.open(self.images[index])
            #img = img.resize((640,480)) #ccsmm, as a  2-tuple:(width,height)

            ## For gray image
            # print(index,np.array(img).shape) #(480,640,3)
            #if np.array(img).shape[-1] is not 3: #bug fix for bad image file, ccsmm, to make 3 channel
            #    img1=np.array(img)
            #    img1[:,:,1]=img1[:,:,0]
            #    img1[:,:,2]=img1[:,:,0]
            #    img = Image.fromarray(img1)
        except:
            #print("Broken image : ", self.images[index])
            return torch.zeros(1), index  # for broken image, set the size of torch to 1 in order to check in for loop.
        if self.input_transform:
            img = self.input_transform(img)
        return img, index

    def __len__(self):
        return len(self.images)

    def getPositives(self):
        # positives for evaluation are those within trivial threshold range
        #fit NN to find them, search by radius
        if  self.positives is None:
            knn = NearestNeighbors(n_jobs=-1)
            knn.fit(self.ImgStruct.utm)

            self.distances, self.positives = knn.radius_neighbors(self.ImgStruct.utm,
                    radius=self.ImgStruct.posDistThr)

        return self.positives

        
def collate_fn(batch):
    """Creates mini-batch tensors from the list of tuples (query, positive, negatives).
    
    Args:
        data: list of tuple (query, positive, negatives). 
            - query: torch tensor of shape (3, h, w).
            - positive: torch tensor of shape (3, h, w).
            - negative: torch tensor of shape (n, 3, h, w).
    Returns:
        query: torch tensor of shape (batch_size, 3, h, w).
        positive: torch tensor of shape (batch_size, 3, h, w).
        negatives: torch tensor of shape (batch_size, n, 3, h, w).
    """

    batch = list(filter (lambda x:x is not None, batch))
    if len(batch) == 0: return None, None, None, None, None

    query, positive, negatives, indices = zip(*batch)

    query = data.dataloader.default_collate(query)
    positive = data.dataloader.default_collate(positive)
    negCounts = data.dataloader.default_collate([x.shape[0] for x in negatives])
    negatives = torch.cat(negatives, 0)
    import itertools
    indices = list(itertools.chain(*indices))

    return query, positive, negatives, negCounts, indices


if __name__ == "__main__":
    #datasetDir = './netvlad_etri_datasets'
    datasetDir = root_dir
    db_dir = os.path.join(datasetDir, 'dbImg')
    queries_dir = os.path.join(datasetDir, 'qImg')

    dbMat_fname = 'etri_db.mat'
    qMat_fname = 'etri_query.mat'

    SaveMatFiles(db_dir,queries_dir,dbMat_fname,qMat_fname)
    dbStruct=LoadMatFiles(dbMat_fname,qMat_fname)
