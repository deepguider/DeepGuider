from __future__ import print_function

import base64
import requests
import os
import geojson
import argparse
import sys
import numpy as np
from ipdb import set_trace as bp

#IMG_SERVER_IPADDR = "129.254.87.96"
IMG_SERVER_IPADDR = "localhost"

class ImgServer:
    def __init__(self,ipaddr=IMG_SERVER_IPADDR):
        self.IP=ipaddr
        self.ROUTING_SERVER_URL = 'http://{}:21500/'.format(self.IP)
        self.STREETVIEW_SERVER_URL = 'http://{}:21501/'.format(self.IP)
        self.POI_SERVER_URL = 'http://{}:21502/'.format(self.IP)
        self.SERVER_URL = self.STREETVIEW_SERVER_URL #default


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

    def QuerytoServer(self,json_save = False,outdir='./'):
        req = self.reqdict
        if req['type']=='tile':
            req_str = self.SERVER_URL + '{}/{}/{}'.format(req['type'], req['tile_num_x'], req['tile_num_y'])
        elif req['type']=='wgs':
            req_str = self.SERVER_URL + '{}/{}/{}/{}'.format(req['type'], req['latitude'], req['longitude'], req['radius'])
        elif req['type']=='node':
            req_str = self.SERVER_URL + '{}/{}/{}'.format(req['type'], req['node_id'], req['radius'])
        else:
            print('{} is not a valid request type'.format(req['type']))
            return None

        response = requests.get(req_str)
        response.raise_for_status()
        elapsed_time = response.elapsed.total_seconds()
        self.json_outputs = response.json()
        if json_save:
            self.SaveGeoJson(outdir)

        return self.json_outputs

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
            print("Not suppported request type")
            self.reqdict = None

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


    def SaveImages(self,outdir='./',verbose=0):
        import requests
        res = self.json_outputs
        ports = 10000
        numImgs = np.size(res['features'])
        for i in range(numImgs):
            imgid = res['features'][i]['properties']['id']
            request_cmd = 'http://{}:{}/{}/{}'.format(self.IP,ports,imgid,'f') #'f' means forward
            fname = os.path.join(outdir,'{}.jpg'.format(imgid))
            r = requests.get(request_cmd,allow_redirects=True)
            open(fname, 'wb').write(r.content)
            if verbose:
                print('{} saved'.format(fname))


    def SaveImage(self,imgid):
        import requests
        ports = 10000
        request_cmd = 'http://{}:{}/{}'.format(self.IP,ports,imgid)
#        response = requests.get('http://ip.address.to.image.server/:port/29300503300')
        fname = '{}.jpg'.format(imgid)
        r = requests.get(request_cmd,allow_redirects=True)
        open(fname, 'wb').write(r.content)
        print('{} saved'.foramt(fname))

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


if __name__ == '__main__':
## Prepare Parameter for Server
    ipaddr = IMG_SERVER_IPADDR #ip address of server
    server_type = "streetview" #server type, layer, or Port
    req_type = "wgs" #coordinate

## Initialize Image Server
    isv = ImgServer(ipaddr)
    isv.SetServerType(server_type)

## Set Position you want to view
# 36.3851418,127.3768362 Lat/Lon near SK-View
# 37.5148562,127.0551826 Boneunsa

    if True: #near ETRI
        gps_lat = 36.3851418
        gps_long = 127.3768362
    else: #near Bongeunsa
        gps_lat = 37.5148562
        gps_long = 127.0551826

    roi_radius = 250.0 #around radius meter
    isv.SetParamsWGS(gps_lat,gps_long,roi_radius)
    isv.SetReqDict(req_type)

## Request to Server    
    res = isv.QuerytoServer(json_save = True,outdir='./')

## for debugging purpose
    numImgs = isv.GetNumImgs()
    if numImgs >0:
        imgID,imgLat,imgLong,imgDate,imgHeading,numImgs = isv.GetStreetViewInfo(0)
#        isv.SaveImage(imgID)
        isv.SaveImages()
