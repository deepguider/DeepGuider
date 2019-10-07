#import sys;sys.path.insert(0,'/home/ccsmm/workdir/ccsmmutils');import img_utils as myiu

import cv2
import numpy as np
import matplotlib.pyplot as plt
from ipdb import set_trace as BP
from PIL import Image

def image_crop( infilename , save_path):
    """
    image file 와 crop한이미지를 저장할 path 을 입력받아 crop_img를 저장한다.
    :param infilename:
        crop할 대상 image file 입력으로 넣는다.
    :param save_path:
        crop_image file의 저장 경로를 넣는다.
    :return:
    """
 
    img = Image.open( infilename )
    (img_h, img_w) = img.size
    print(img.size)
 
    # crop 할 사이즈 : grid_w, grid_h
    grid_w = 96 # crop width
    grid_h = 96 # crop height
    range_w = (int)(img_w/grid_w)
    range_h = (int)(img_h/grid_h)
    print(range_w, range_h)
 
    i = 0
 
    for w in range(range_w):
        for h in range(range_h):
            bbox = (h*grid_h, w*grid_w, (h+1)*(grid_h), (w+1)*(grid_w))
            print(h*grid_h, w*grid_w, (h+1)*(grid_h), (w+1)*(grid_w))
            # 가로 세로 시작, 가로 세로 끝
            crop_img = img.crop(bbox)
 
            fname = "{}.jpg".format("{0:05d}".format(i))
            savename = save_path + fname
            crop_img.save(savename)
            print('save file ' + savename + '....')
            i += 1
 

def concat_images(imga, imgb):
    """
    Combines two color image ndarrays side-by-side.
    """
    imga=imgreshape(imga)
    imgb=imgreshape(imgb)

    ha,wa = imga.shape[:2]
    hb,wb = imgb.shape[:2]
    max_height = np.max([ha, hb])
    total_width = wa+wb
    new_img = np.zeros(shape=(max_height, total_width, 3))
    new_img[:ha,:wa]=imga
    new_img[:hb,wa:wa+wb]=imgb
    return new_img

def concat_n_images(image_path_list):
    """
    Combines N color images from a list of image paths.
    """
    output = None
    for i, img_path in enumerate(image_path_list):
        img = plt.imread(img_path)[:,:,:3]
        if i==0:
            output = img
        else:
            output = concat_images(output, img)
    return output

def tonumpy(data):
    import numpy as np
    import torch
    try: data.type()
    except :
        return data
#    if ( ('torch' in type(data)) and ('Tensor' in type(data)) ):
#        if ('cuda' in type(data)):
    if ( ('torch' in data.type()) and ('Tensor' in data.type()) ):
        if ('cuda' in data.type()):
            data=data.detach().cpu().numpy()
        else:
            data=data.detach().numpy()
    return data

def subplot(sp=111):
    plt.subplot(sp)

def title(title="No title"):
    plt.title(title)

def plot(x,sp=111,title="No title",t=0.000001,fdir='./',save=0):
    import matplotlib.pylab as plt
    import numpy as np
    import os
    import torch

    if len(np.shape(x)) != 1:
        print("dimension not matched, getting last element")
        x = x[-1]
#        return 0

    data = tonumpy(x)

    plt.subplot(sp)
    plt.plot(data)
    plt.title(title)

    if(t>0):
        plt.draw()
        plt.pause(t)
    else:
        plt.show()
    if save:
        fname=os.path.join(fdir,title+'.png')
        plt.savefig(fname,bbox_inches='tight')

def imshow(img,sp=111,title="No title",t=0.000001,fdir='./',save=0):
    import matplotlib.pylab as plt
    import numpy as np
    import os
    import torch
    if len(np.shape(img)) == 4:   #batch_size * rgb
        img=img[-1] #Take last image(3,480,640)
    elif len(np.shape(img)) == 3: #rgb
        img=img
    elif len(np.shape(img)) == 2: #gray
        img=img
    else:
        print("dimension not matched")
        return 0
    if isinstance(img, torch.Tensor):
        img=tonumpy(img)
        img=imgreshape(img)
    plt.subplot(sp)
    plt.imshow(img)
    plt.title(title)
    
    if(t>0):
        plt.draw()
        plt.pause(t)
    else:
        plt.show()
    if save:
        fname=os.path.join(fdir,title+'.png')
        plt.savefig(fname,bbox_inches='tight')

def fig(num=0):
    if num:
        plt.figure(num)
    else:
        plt.figure()


def cla():
    plt.cla()

def clf():
    plt.clf()

def imsshow(imgs):
    import torchvision
    imshow(torchvision.utils.make_grid(imgs))

def imgreshape(img):
    """img must be a tensor type data"""
    import numpy as np
    import torch
    if isinstance(img,torch.Tensor):
        img=img.squeeze()
    if len(img.shape) == 2:
#        img=img.unsqueeze(0)
        np.reshape(img,[1,np.shape(img)[0],np.shape(img)[1]])

    if len(img.shape) == 3:
        if img.shape[0] <= 3:
            img=np.transpose(img,(1,2,0))

    return img

def imgnormalize(img):
    import cv2
    return cv2.normalize(img,None,0,255,cv2.NORM_MINMAX)


#Default color space in OpenCV is BGR, but matplotlib's is RGB.
#So when we use matplotlib to disply it, we need to change the color space from bgr to rgb

def bgr2rgb(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def rgb2bgr(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def test_dispgen(model,imgL,imgR,idx):
        model.eval()


#        imgL = torch.FloatTensor(imgL).cuda()
#        imgR = torch.FloatTensor(imgR).cuda()    
        imgL = imresize(imgL,(368,1232))
        imgR = imresize(imgR,(368,1232))

        imgL=imgL.astype(np.float32)/256
        imgR=imgR.astype(np.float32)/256

        imgL = imgL.transpose(2,0,1)
        imgR = imgR.transpose(2,0,1)

        imgL=imgL.reshape(1,3,368,1232)
        imgR=imgR.reshape(1,3,368,1232)

        imgL = Variable(torch.FloatTensor(imgL).cuda())
        imgR = Variable(torch.FloatTensor(imgR).cuda())

        with torch.no_grad():
            output = model(imgL,imgR)
        output = torch.squeeze(output)
        pred_disp = output.data.cpu().numpy()
        display_save(imgL,imgR,pred_disp,idx)

        return pred_disp


def display_save(imgL,imgR,dispL,inx):
    import os
    import skimage
    import skimage.io
    import skimage.transform
#    output_dir='output_eval_disp'
    output_dir=argsglb.output_dir
    plt.clf()
    img_utils.imshow(imgL,sp=221,title='Left')
    img_utils.imshow(imgR,222,'Right')
    img_utils.imshow(dispL.astype('uint16'),223,'Disp(est)')
#    img_utils.imshow(dispL_GT.astype('uint16'),224,'Disp(GT)')
    fname=os.path.join(output_dir,'psmnet_all_{}'.format(inx))
    plt.savefig(fname,bbox_inches='tight')
    fname=os.path.join(output_dir,'psmnet_disp_{}.png'.format(inx))
#    skimage.io.imsave(fname,(dispL.squeeze()*256).astype('uint16'))
    return 0
