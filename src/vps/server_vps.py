from flask import Flask, request, Response, render_template
from werkzeug import secure_filename
import jsonpickle
import json
import numpy as np
import cv2
import os
from ipdb import set_trace as bp
from time import sleep
import matplotlib.pyplot as plt

from vps import vps

## Some misc. functions
def serialize_image(image): # ndarray h*w*c
    image_size = image.shape
    [h, w, c] = image_size
    image_data = image.reshape(h*w*c).tolist() # serialize and change to list
    return image_size, image_data


def deserialize_image(image_data, image_size): # ndarray 1d
    [h, w, c] = image_size
    return  np.asarray(image_data).reshape(h, w, c) # de-serialize


def dummy_apply(image, K, gps_lat, gps_lon, gps_accuracy, timestamp):
    vps_imgID = [int(i) for i in (100*np.random.rand(3)).astype(int) ] # list of uint64, fixed dg'issue #36
    vps_imgConf = [float(i) for i in np.random.rand(3)] # list of double(float64), fixed dg'issue #36
    vps_IDandConf = [vps_imgID, vps_imgConf]
    return vps_IDandConf


## Initialize the Flask application
app = Flask(__name__)


# route http posts to this method
@app.route('/UploadFile/', methods=['GET', 'POST'])
def UploadFile():
    path = 'StreetView'
    r = request
    if r.method == 'POST':
        f = r.files['file']
        filename = os.path.join(path, secure_filename(f.filename))
        f.save(filename)
        response_dict = {
                'message': filename}
        response_pickled = jsonpickle.encode(response_dict)
        return Response(response=response_pickled, status=200, mimetype="application/json")


@app.route('/SendData/', methods=['GET', 'POST'])
def SendData():
    r = request
    if r.method == 'POST':
        jsondata = r.get_json()
        rcv_data = json.loads(jsondata)
        K = rcv_data['K']
        gps_lat = rcv_data['gps_lat']
        gps_lon = rcv_data['gps_lon']
        gps_accuracy = rcv_data['gps_accuracy']
        [h, w, c] = rcv_data['image_size']
        image_data = rcv_data['image_data']
        image = np.asarray(image_data).reshape(h, w, c) # de-serialize
        streetview_server_ipaddr = rcv_data['streetview_server_ipaddr']
        print(K, gps_lat, gps_lon, gps_accuracy, streetview_server_ipaddr)
        response = rcv_data # return back received data to client for debugging in client side
        response_pickled = jsonpickle.encode(response)
        return Response(response=response_pickled, status=200, mimetype="application/json")


@app.route('/SendImage/', methods=['GET', 'POST'])
def SendImage():
    path = 'StreetView'
    r = request
    if r.method == 'POST':
        nparr = np.frombuffer(r.data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        response = {'message': 'image received. size={}x{}'.format(img.shape[1], img.shape[0])}
        response_pickled = jsonpickle.encode(response)
        return Response(response=response_pickled, status=200, mimetype="application/json")


## Callback for VPS
@app.route('/Apply/', methods=['GET', 'POST']) # for vps apply
def Apply():
    global mod_vps, apply_response
    r = request
    if r.method == 'POST':
        jsondata = r.get_json()
        if type(jsondata) is str:
            rcv_data = json.loads(jsondata)
        else:
            rcv_data = jsondata
        # type(rcv_data) must be dict

        if type(rcv_data) is not dict:
            print("Invalid post data type from client")
            apply_response = {"vps_IDandConf" : [[0],[0]] }
            response_pickled = jsonpickle.encode(apply_response)
            return Response(response = response_pickled, status=200, mimetype="application/json")
            
        K = rcv_data['K']
        gps_lat = rcv_data['gps_lat']
        gps_lon = rcv_data['gps_lon']
        gps_accuracy = rcv_data['gps_accuracy']
        image_data = rcv_data['image_data']
        image_size = rcv_data['image_size']
        query = deserialize_image(image_data, image_size)
        timestamp = rcv_data['timestamp']
        streetview_server_ipaddr = rcv_data['streetview_server_ipaddr']
        # print(K, gps_lat, gps_lon, gps_accuracy, streetview_server_ipaddr)
        try:
            # vps_IDandConf = dummy_apply(image, K, gps_lat, gps_lon, gps_accuracy, 0)
            vps_IDandConf = mod_vps.apply(query, K, gps_lat, gps_lon, gps_accuracy, timestamp, ipaddr = streetview_server_ipaddr)
        except:
            bp()
        apply_response = {"vps_IDandConf" : vps_IDandConf, 'timestamp' : timestamp}
        response_pickled = jsonpickle.encode(apply_response)
        return Response(response = response_pickled, status=200, mimetype="application/json")
    if r.method == 'GET':
        if not 'apply_response' in globals():
            apply_response = {"vps_IDandConf" : np.zeros((2,3)).tolist(), 'timestamp' : 0}
        response_pickled = jsonpickle.encode(apply_response)
        return Response(response = response_pickled, status=200, mimetype="application/json")


if __name__ == '__main__':
    ## Initialize the VPS module
    global mod_vps
    mod_vps = vps()
    if mod_vps.initialize() < 0:
        print("Error : vps.initialize() ")
    else:
        app.run(host="localhost", port=7729, threaded=True)
