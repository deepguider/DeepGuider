from __future__ import print_function
import requests
import json
import cv2
import numpy as np
from ipdb import set_trace as bp


## Misc ..
def imshow(title, image, sp = '111'):
    import matplotlib.pyplot as plt
    plt.subplot(sp)
    plt.imshow(image) # rgb
    plt.title(title)
    plt.axis("off")
    plt.draw()
    plt.pause(0.1)


def get_myDict():
    import random
    import cv2
    import numpy as np
    K = 3 
    h, w, c = 3, 4, 3
    img = np.zeros([h, w, c]) 
    img[:,:,0] = np.ones([h, w])*random.choice([0,255]) # h, w, c
    img[:,:,1] = np.ones([h, w])*random.choice([0,255]) # h, w, c
    img[:,:,2] = np.ones([h, w])*random.choice([0,255]) # h, w, c
    img=img.astype(int)
    image_size = [i for i in np.shape(img)] # h, w, c, tuple, list <==> list through json
    image_data = img.reshape(h*w*c).tolist() # serialize and change to list
    gps_lat = 36.381438
    gps_lon = 127.378867
    gps_accuracy = 0.9 
    streetview_server_ipaddr = "localhost"
    data_dict = { 'K' : K, 'image_size' : image_size, 'image_data' : image_data,
            'gps_lat' : gps_lat, 'gps_lon' : gps_lon, 'gps_accuracy' : gps_accuracy,
            'streetview_server_ipaddr' : streetview_server_ipaddr }
    ## display for debugging
    img_recon = np.asarray(image_data).reshape(h, w, c) # de-serialize
    imshow('Client : send image', img_recon, '121')
    return data_dict




def compare_Dict(a_dict, b_dict):
    ret = True
    for key, val in a_dict.items():
        if val != b_dict[key]:
            print("Senddata differ from receviced data",val,b_dict[key])
            ret = False
            return ret
    return ret


## Function definition for request command 

def client_UploadFile(url, filename):
    cmd_url = url + '/UploadFile/' # You have to implement @app.route('/Apply/') in server.py part
    fin = open(filename, 'rb')
    files ={'file': fin}
    try:
        r = requests.post(cmd_url, files=files)
        #print(json.loads(r.text))
        print(r.text)
        fin.close()
        return 0
    except:
        fin.close()
        return -1


def client_SendImage(url, filename):
    cmd_url = url + '/SendImage/'
    return 0


def client_SendData(url): # data must be dictionary type
    data_dict = get_myDict()
    cmd_url = url + '/SendData/' # You have to implement @app.route('/SendData/') in server.py part
    data_json = json.dumps(data_dict)
    rcv_json = requests.post(cmd_url, json=data_json) # json data
    rcv_dict = rcv_json.json() #dictionary
    #rcv_dict = json.load(rcv_json) #dictionary

    ## Verfication
    if compare_Dict(data_dict, rcv_dict):
        print("Success comparing send and recevived data")
    
    image_data = rcv_dict['image_data']
    [h, w, c] = rcv_dict['image_size']
    img_recon = np.asarray(image_data).reshape(h, w, c) # de-serialize
    imshow('Client : rcv image', img_recon, '122')


def serialize_image(image): # ndarray h*w*c
    image_size = image.shape
    [h, w, c] = image_size
    image_data = image.reshape(h*w*c).tolist() # serialize and change to list
    return image_size, image_data


def deserialize_image(image_data, image_size): # ndarray 1d
    [h, w, c] = image_size
    return  np.asarray(image_data).reshape(h, w, c) # de-serialize


def client_Apply(url, aviname, streetview_server_ipaddr): # data must be dictionary type
    cmd_url = url + '/Apply/' # You have to implement @app.route('/Apply/') in server.py part
    cap = cv2.VideoCapture(aviname)

    frame_cnt = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        if not ret :
            continue
        frame_cnt += 1

        if (frame_cnt % 30) != 0:
            continue
            
        K = 3 
        frame = frame.astype(int)
        frame = frame[:, :, [2, 1, 0]] # bgr to rgb
        image_size, image_data = serialize_image(frame)

        gps_lat = 36.381438 + 0.00001
        gps_lon = 127.378867 + 0.00001
        gps_accuracy = 1 
        data_dict = { 'K' : K, 'image_size' : image_size, 'image_data' : image_data,
            'gps_lat' : gps_lat, 'gps_lon' : gps_lon, 'gps_accuracy' : gps_accuracy,
            'streetview_server_ipaddr' : streetview_server_ipaddr }
        data_json = json.dumps(data_dict)
        rcv_json = requests.post(cmd_url, json=data_json) # input : json data, it takes some second to return
        rcv_dict = rcv_json.json() # .json method changes json to dictionary
        vps_image_data = rcv_dict['vps_IDandConf']
        print(frame_cnt, vps_image_data)

        ## display for debugging
        img_recon = deserialize_image(image_data, image_size)
        imshow('Client : query image', img_recon, '111')
        sleep(0.1)


if __name__ =='__main__':
    from time import sleep
    Debug = False
    streetview_server_ipaddr = "localhost"
    vps_server_addr='http://localhost:7729'
    filename = '63050401326.jpg' # query file name
    aviname = '/home/ccsmm/Naverlabs/query_etri_cart/191115_ETRI.avi'

    url = vps_server_addr
    if Debug :
        for i in range(1000):
            client_UploadFile(url, filename) # Ok
            client_SendData(url) # Ok
            sleep(1)
    else:
        client_Apply(url, aviname, streetview_server_ipaddr)
