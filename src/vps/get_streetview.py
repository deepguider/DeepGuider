from __future__ import print_function

import base64
import requests
import os
import geojson
import json
import argparse
import sys
import numpy as np

from ipdb import set_trace as bp
from dmsg import dmsg

class ImgServer:
    def __init__(self,ipaddr="localhost"): # 127.0.0.1
        self.IP=ipaddr
        self.ROUTING_SERVER_URL = 'http://{}:21500/'.format(self.IP)
        self.STREETVIEW_SERVER_URL = 'http://{}:21501/'.format(self.IP)
        self.POI_SERVER_URL = 'http://{}:21502/'.format(self.IP)
        self.SERVER_URL = self.STREETVIEW_SERVER_URL #default
        self.req_type='wgs'
        self.gps_lat=37.513366
        self.gps_long=127.056132
        self.roi_radius=100
        self.reqdict = {'type': 'wgs', 'latitude': self.gps_lat, 'longitude': self.gps_long, 'radius': self.roi_radius}

    def SetServerType(self,server_type):
        self.server_type=server_type
        if server_type=="routing":
            self.SERVER_URL = self.ROUTING_SERVER_URL
        elif server_type=="streetview":
            self.SERVER_URL = self.STREETVIEW_SERVER_URL
        elif server_type=="poi":
            self.SERVER_URL = self.POI_SERVER_URL
        else:
            self.SERVER_URL = self.STREETVIEW_SERVER_URL
        return 0

    def SystemCall(self,cmd_str): # "curl http://google.com"
        import subprocess
        ret = subprocess.check_output (cmd_str + " 2>&1", shell=True)
        return ret

    def QuerytoServer(self, json_save = False, outdir='./', PythonOnly=False):
        req = self.reqdict
        if req['type']=='tile':
            req_str = self.SERVER_URL + '{}/{}/{}'.format(req['type'], req['tile_num_x'], req['tile_num_y'])
        elif req['type']=='wgs':
            req_str = self.SERVER_URL + '{}/{}/{}/{}'.format(req['type'], req['latitude'], req['longitude'], req['radius'])
        elif req['type']=='node':
            req_str = self.SERVER_URL + '{}/{}/{}'.format(req['type'], req['node_id'], req['radius'])
        else:
            print('{} is not a valid request type'.format(req['type']))
            return -1
        try:
            if PythonOnly: # Code runs in "Python only" Environment
                import requests
                timeout=(2, 2) # timeout of (connect, read) sec.
                response = requests.get(req_str,timeout=timeout) # may cause seg.fault in (C+Python Environ.)
                response.raise_for_status() # will raise error if return code is not 200 (meaning OK)
                elapsed_time = response.elapsed.total_seconds()
                self.json_outputs = response.json()
            else:  # Code runs in "C++ + Python" environment
                # set timeout 2 sec.
                # bash command : curl --silent --connect-timeout 3 http://localhost:21501/wgs/36.381448000000006/127.378867/30 
                cmd_str = "curl --silent --connect-timeout 2 " + req_str
                ret = self.SystemCall(cmd_str)
                ret_json = json.loads(ret)
                self.json_outputs = ret_json
        except:
            return -1
        if json_save and (self.json_outputs != None):
            self.SaveGeoJson(outdir)
        return 0

    def GetReqDict(self):
        return self.reqdict

    def SetReqDict(self,req_type):
        self.req_type = req_type
        if req_type=="tile":
            self.reqdict = {'type': 'tile', 'tile_num_x': 55897, 'tile_num_y': 25393}
        elif req_type=="wgs":
            self.reqdict = {'type': 'wgs', 'latitude': self.gps_lat, 'longitude': self.gps_long, 'radius': self.roi_radius}
            #req = {'type': 'wgs', 'latitude': 30, 'longitude': 120} # Invalid request (out of range)
        elif req_type=="node":
            self.reqdict = {'type': 'node', 'node_id': 558992539200991, 'radius': 500.0} # Valid request
            #req = {'type': 'node', 'node_id': 1000} # Invalid request
        else:
            print("Not suppported request type. Set to wgs")
            self.reqdict = self.reqdict = {'type': 'wgs', 'latitude': self.gps_lat, 'longitude': self.gps_long, 'radius': self.roi_radius}
        return self.reqdict

    def SetParamsWGS(self,gps_lat=37.513366,gps_long=127.056132,roi_radius=10.0):
        self.gps_lat = gps_lat
        self.gps_long = gps_long
        self.roi_radius = roi_radius

    def SaveGeoJson(self,outdir='./'):
        res = self.json_outputs
        if res != None:
            fname = os.path.join(outdir,"{}_{}.geojson".format(self.server_type, self.req_type)) 
            with open(fname,'w') as f:
                #print('{} saved'.format(fname))
                geojson.dump(res, f)

    def SaveImages(self, outdir='./', verbose=0, PythonOnly=False):
        import requests
        import os
        res = self.json_outputs
        ports = 10000
        numImgs = np.size(res['features'])
        for i in range(numImgs):
            imgid = res['features'][i]['properties']['id']
            request_cmd = 'http://{}:{}/{}/{}'.format(self.IP,ports,imgid,'f') #'f' means forward
            fname = os.path.join(outdir,'{}.jpg'.format(imgid))
            try:
                if PythonOnly: # Code runs in "Python only" Environment
                    r = requests.get(request_cmd,allow_redirects=True)
                    open(fname, 'wb').write(r.content)
                else:  # Code runs in "C++ + Python" environment
                    # set timeout 2 sec.
                    # curl http://ip.address.to.image.server:port/29300503300 --output test.jpg
                    curl_cmd ="curl --silent --connect-timeout 2 " + request_cmd + " --output " + fname
                    os.system(curl_cmd)
                    #self.SystemCall(curl_cmd)
            except:
                return -1
            if verbose:
                print('{} saved'.format(fname))
        return 0

    def GetNumImgs(self):
        res = self.json_outputs
        numImgs = np.size(res['features'])
        return numImgs

    def GetStreetViewInfo(self,imgidx):
        res = self.json_outputs
        numImgs = np.size(res['features'])
        imgLong = res['features'][imgidx]['properties']['longitude']
        imgLat = res['features'][imgidx]['properties']['latitude']
        imgDate = res['features'][imgidx]['properties']['date']
        imgHeading = res['features'][imgidx]['properties']['heading']
        imgID = res['features'][imgidx]['properties']['id']
        return imgID,imgLat,imgLong,imgDate,imgHeading,numImgs

def makedir(fdir):
    import os
    if not os.path.exists(fdir):
        os.makedirs(fdir)

def GetStreetView(gps_lat,gps_long,roi_radius=100,ipaddr='localhost',server_type="streetview",
        req_type="wgs",outdir='./'):
    ## Input
    # gps_lat/long : latitude and longitude of gps
    # roi_radius : radius value to download the image around the current coordinate
    # outdir : directory to save the downloaded images
    # ipaddr : ip address of server
    # server_type : "streetview" which is server type, layer, or Port
    # req_type : "wgs" which is coordinate

    ## Initialize image server
    isv = ImgServer(ipaddr)
    isv.SetServerType(server_type)
    
    ## Set position you want to view and request type
    isv.SetParamsWGS(gps_lat,gps_long,roi_radius) # 37,27,100
    isv.SetReqDict(req_type)

    ## Request to Server ( possibility of seg fault )
    ret = isv.QuerytoServer(json_save=False, outdir=outdir, PythonOnly=False)
    if ret == -1:
        #raise Exception('Image server is not available.')
        print('Image server is not available.')
        return -1

    ## Save downloaded image at outdir
    numImgs = isv.GetNumImgs()
    if numImgs >0: 
        imgID,imgLat,imgLong,imgDate,imgHeading,numImgs = isv.GetStreetViewInfo(0)
        ret = isv.SaveImages(outdir)
        if ret == -1:
            print('Image server is not available.')
            return -1

if __name__ == '__main__':
    ## Prepare Parameter for Server

    ## Set Position you want to view
    if True: #near ETRI(SK-View Apt.)
        gps_lat = 36.3851418
        gps_long = 127.3768362
    else: #near Bongeunsa
        gps_lat = 37.5148562
        gps_long = 127.0551826
    roi_radius = 250.0 # boundary radius in meter

    outdir='./download_jpg'
    makedir(outdir)
    for i in range(1000):
        print(str(i) + '-th request')
        GetStreetView(gps_lat, gps_long, roi_radius, 
                ipaddr='localhost', server_type="streetview",
                req_type="wgs",outdir=outdir)
