from __future__ import print_function
import os
import glob # may cause segmentation fault in (C+Python) environment
import numpy as np
import cv2
import csv
import faiss
import pandas as pd
import utm

### For dataset
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset
from torch.utils.data import DataLoader

import get_streetview
from vps import vps
import unit_test_vps_config as config
import sys;sys.path.insert(0,'/home/ccsmm/workdir/ccsmmutils');import torch_img_utils as tim
from ipdb import set_trace as bp

postfix_dict = {
        "ascen_fix":"ascen_fix.csv", 
        "images":"images.avi",
        "imu_data":"imu_data.csv",
        "intersect":"intersect.csv",
        "novatel_fix":"novatel_fix.csv"}

def get_input_file_list(testset_dir, ext="*.avi", out_postfix='vps.csv'):
    flist = glob.glob(os.path.join(testset_dir, ext))
    flist.sort()
    images = []
    ascen_fix = []
    outputs = []
    for i, fn in enumerate(flist):
        prefix = os.path.basename(fn)[:14]  # "191115_151140_" from 191115_151140_images.avi
        #images.append(os.path.join(testset_dir, prefix+postfix_dict["images"]))
        images.append(fn)
        ascen_fix.append(os.path.join(testset_dir, prefix+postfix_dict["ascen_fix"]))
        outputs.append(os.path.join(testset_dir, prefix+out_postfix))
    assert len(images)==len(ascen_fix), "Number of files are mis-matched."
    return images, ascen_fix, outputs

class dgDataset(Dataset):
    def __init__(self, avi, ascen):
        self.cap = cv2.VideoCapture(avi)
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.video_frame_length = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
        
        self.timestamp, self.lat, self.lon, self.alt = self.parse_ascen(ascen)
        self.starttime = self.timestamp[0]
        self.endtime = self.timestamp[-1]
        self.time_length = self.endtime - self.starttime  # ascen's total seconds
        self.video_scale = self.video_fps * self.time_length / self.video_frame_length

        print("=====> Start reading {} and {}.".format(avi, ascen))

    def parse_ascen(self, ascen):
        '''
        ipdb> data.head()
                                 time  .header.seq  .header.stamp.secs  .header.stamp.nsecs .header.frame_id  ...  .latitude  .longitude  .altitude                               .position_covariance  .position_covariance_type
        0  2019/11/15/11:40:06.994837           24          1573785606            994514942              gps  ...  36.382057  127.367646       90.2  (18.3184, 0.0, 0.0, 0.0, 18.3184, 0.0, 0.0, 0....                          1
        1  2019/11/15/11:40:07.993330           25          1573785607            993129014              gps  ...  36.382056  127.367646       90.5  (18.3184, 0.0, 0.0, 0.0, 18.3184, 0.0, 0.0, 0....                          1
        2  2019/11/15/11:40:08.991022           26          1573785608            990794897              gps  ...  36.382057  127.367646       90.4  (24.2064, 0.0, 0.0, 0.0, 24.2064, 0.0, 0.0, 0....                          1
        '''
        data = pd.read_csv(ascen, sep=",")
        lat = np.array([ float(i) for i in data[".latitude"]])
        lon = np.array([ float(i) for i in data[".longitude"]])
        alt = np.array([ float(i) for i in data[".altitude"]])
        timestamp = np.array([float(i) for i in data[".header.stamp.secs"]])
        return timestamp, lat, lon, alt

    def get_image_timestamp(self, fnumber):
        timestamp = self.starttime + fnumber * self.video_scale / self.video_fps
        return timestamp

    def get_latlon_from_timestamp(self, q_timestamp):
        best_similar_idx = np.argmin(np.abs(self.timestamp - q_timestamp))
        return self.lat[best_similar_idx], self.lon[best_similar_idx], best_similar_idx

    def __len__(self): 
        return int(self.video_frame_length )

    def release(self):
        self.cap.release()

    def __getitem__(self, idx):
        fnumber = idx
        ret, qimg = self.cap.read()
        image_timestamp = self.get_image_timestamp(fnumber)
        lat, lon, tidx = self.get_latlon_from_timestamp(image_timestamp)
        return [qimg, fnumber, image_timestamp, lat, lon]

def get_utm_err(lat1, lon1, lat2, lon2):
     if np.isnan(lat1) or np.isnan(lat1) or np.isnan(lon1) or np.isnan(lon2):
         return -1
     if lat1 < 36 or lon1 < 127 or lat2 < 36 or lon2 < 127:
         return -1
     if lat1 > 38 or lon1 > 128 or lat2 > 38 or lon2 > 128:
         return -1
     p1 = np.array(utm.from_latlon(lat1, lon1)[0:2])
     p2 = np.array(utm.from_latlon(lat2, lon2)[0:2])
     err_l2norm = np.linalg.norm(p1-p2) # l2norm = np.sqrt(np.sum((p1-p2)**2))
     return err_l2norm

def do_vps(avi, ascen, output_filename,  begin_frame=1000, server_ip="129.254.81.204"):  # file names
    print("=====> Start reading {}.".format(avi))
    dataset = dgDataset(avi, ascen)
    dataloader = DataLoader(dataset, batch_size=1, shuffle=False)

    fout = open(output_filename, 'w', buffering=1)

    string='fnumber,timestamp,svid,svidx,pred_lat,pred_lon,distance,angle,confidence,curr_lat,curr_lon,utm_err'
    fout.write(string+'\n')
    print(string)

    for idx, [qimg, fnumber, timestamp, lat, lon] in enumerate(dataloader):
        qimg = qimg.numpy()[0]
        fnumber = fnumber.numpy()[0]
        timestamp = timestamp.numpy()[0]
        curr_lat = lat.numpy()[0]
        curr_lon = lon.numpy()[0]
        try:
            [h, w, c] = qimg.shape
            if (h < 480) or (w < 640) or c != 3:
                print("Invalid shape of query image :", h,w,c)
                continue
        except:
            print("Broken query image :", fname)
            continue
        qimg = cv2.resize(qimg,(640,480))
        #vps_IDandConf = mod_vps.apply(qimg, 3, 36.381438, 127.378867, 0.8, 1.0, streetview_server_ipaddr) # k=5 for knn
        if idx < begin_frame:  # Skip beginning videos
            print("Skip {}\r".format(idx), end='')
            continue
        #cv2.imshow('QImg', qimg)
        #cv2.waitKey(1)
        vps_IDandConf = mod_vps.apply(image=qimg, K=1, gps_lat=lat, gps_lon=lon, gps_accuracy=0.8, timestamp=timestamp, ipaddr=server_ip) # k=5 for knn
        svid = vps_IDandConf[0][0]  # street view id from map server
        svidx = "f"  # cubic == "f" || cubic == "b" || cubic == "l" || cubic == "r" || cubic == "u" || cubic == "d")
        confidence = vps_IDandConf[1][0]  # 0 ~ 1, default 1
        distance = -1  # distance in the ground from camera to predicted point. default -1, meter
        angle = np.pi  # Relative angle from camera to predicted point(CCW : +). default is pi, radian
        _, pred_lat, pred_lon = get_streetview.GetStreetView_fromID(svid, roi_radius=1, ipaddr=server_ip)
        
        utm_err = get_utm_err(curr_lat, curr_lon, pred_lat, pred_lon)

        string = '{0:04d},{1:10.3f},{2:11d},{3:},{4:2.8f},{5:3.7f},{6:3d},{7:1.3f},{8:1.3f},{9:2.8f},{10:3.7f},{11:3.1f}'.format(
            fnumber,timestamp,svid,svidx,pred_lat,pred_lon,distance,angle,confidence,curr_lat,curr_lon,utm_err)
        fout.write(string+'\n')
        print(string)

 #       print('{0:04d},{1:10.0f},{2:11d},{3:},{4:2.8f},{5:3.7f},{6:3d},{7:1.3f},{8:1.3f},{9:2.8f},{10:3.7f},{11:3.1f}'.format(fnumber,timestamp,svid,svidx,pred_lat,pred_lon,distance,angle,confidence,curr_lat,curr_lon,utm_err))
        if False:
            ## Display Result
            qImgs  = mod_vps.get_qImgs() #  [10,3,480,640] 
            dbImgs = mod_vps.get_dbImgs() #  [10,3,480,640] 
            qdbImgs = torch.cat((qImgs,dbImgs),-1) #  [10,3,480,1280] 

    fout.close()
    dataset.release()
    #cv2.destroyAllWindows()

if __name__ == "__main__":
    ## Image server address
    server_ip = config.ip

    ## Set the GPU number (which gpu will you use)
    gpu_num = config.which_gpu

    ## In/out directory and file information
    testset_dir = config.indir
    in_ext = config.input_reference_ext  # "*.avi"
    out_postfix = config.out_postfix   # "vps_lr.csv"
    date_idx = config.date_idx

    ## Skip frame for invalid video at the very begenning.
    begin_skip_frame = config.begin_skip_frame

    images, ascen_fix, outputs = get_input_file_list(testset_dir, ext=in_ext, out_postfix=out_postfix)
    mod_vps = vps(gpu_num)
    mod_vps.initialize()
    avi_filename = images[date_idx]
    ascen_filename = ascen_fix[date_idx]
    output_filename = outputs[date_idx]
    do_vps(avi_filename, ascen_filename, output_filename, begin_skip_frame, server_ip)  # avi and novatel are filenames

    #for i, [avi, ascen] in enumerate(zip(images, ascen_fix)):
        #do_vps(avi_filename, ascen_filename, output_filename, 2500, server_ip)  # avi and novatel are filenames
