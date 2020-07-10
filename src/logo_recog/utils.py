'''
Modified from Logohunter, https://github.com/ilmonteux/logohunter
'''

import cv2
import os
import h5py
import time
import colorsys
import numpy as np

from keras import Model
from PIL import Image, ImageDraw, ImageFont
from matplotlib.colors import rgb_to_hsv, hsv_to_rgb
from sklearn.metrics.pairwise import cosine_similarity


def draw_matches(image, label_list, prediction, matches):
    '''Draw bounding boxes on image with matching results.'''
    
    if len(prediction) == 0:
        return image

    image = Image.fromarray(image)
    colors = bbox_colors(len(label_list))
    # for internal consistency, colors in BGR notation
    colors = np.array(colors)[:, ::-1]
    
    match_bbox = []
    for i in range(len(label_list)):
        match_bbox.append([])
        for i_cand, (i_match, cdf) in matches.items():
            if i==i_match:
                match_bbox[i].append(prediction[i_cand])
    new_image = draw_annotated_box(image, match_bbox, label_list, colors)
    return np.array(new_image)


def bbox_colors(num_colors):
    '''Select n distinct bounding box colors.'''

    hsv_tuples = [(x / num_colors, 1., 1.) for x in range(num_colors)]
    colors = 255 * np.array([colorsys.hsv_to_rgb(*x) for x in hsv_tuples])

    np.random.seed(1234)
    np.random.shuffle(colors)
    np.random.seed(None)

    return colors.astype(int)


def draw_annotated_box(image, bbox_list, label_list, color_list):
    '''Draw box and overhead label on image.'''

    font_path = os.path.join(os.path.dirname(__file__), 'model/keras_yolo3/font/FiraMono-Medium.otf')
    font = ImageFont.truetype(font=font_path, size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
    thickness = (image.size[0] + image.size[1]) // 300

    draw = ImageDraw.Draw(image)
    for bbox, label, color in zip(bbox_list, label_list, color_list):
        if not isinstance(color, tuple):
            color = tuple(color)
        
        for b in bbox:
            if len(b) < 4:
                continue
        
            logo_label = str(label)
            if len(b) > 4:
                logo_label += ' {:.2f}'.format(b[-1]) # adding confidence
            label_size = draw.textsize(logo_label, font)

            xmin, ymin, xmax, ymax = b[:4]
            xmin = max(0, np.floor(xmin + 0.5).astype('int32'))
            ymin = max(0, np.floor(ymin + 0.5).astype('int32'))
            xmax = min(image.size[0], np.floor(xmax + 0.5).astype('int32'))
            ymax = min(image.size[1], np.floor(ymax + 0.5).astype('int32'))

            if ymin - label_size[1] >= 0:
                text_origin = np.array([xmin, ymin - label_size[1]])
            else:
                text_origin = np.array([xmin, ymax])

            for i in range(thickness):
                draw.rectangle([xmin + i, ymin + i, xmax - i, ymax - i], outline=color)
            draw.rectangle([tuple(text_origin), tuple(text_origin + label_size)], fill=color)
            draw.text(text_origin, logo_label, fill=(0, 0, 0), font=font)
    del draw
    return image


def pad_image(img, shape, mode = 'constant_mean'):
    '''Resize and pad image to given size.'''
    
    if mode == 'constant_mean':
        mode_args = {'mode': 'constant', 'constant_values': np.mean(img)}
    else:
        mode_args = {'mode': mode}

    ih, iw = img.shape[:2]
    h, w = shape[:2]

    # first rescale image so that largest dimension matches target
    scale = min(w/iw, h/ih)
    nw, nh = int(iw * scale), int(ih * scale)
    img = cv2.resize(img, (nw, nh))

    # center-pad rest of image: compute padding and split in two
    xpad, ypad = shape[1]-nw, shape[0]-nh
    xpad = (xpad//2, xpad//2+xpad%2)
    ypad = (ypad//2, ypad//2+ypad%2)

    new_im = np.pad(img, pad_width=(ypad, xpad, (0,0)), **mode_args)

    return new_im


def extract_features(img, model, preprocess, batch_size=100):
    '''Extract features from image array.'''
    
    if len(img) == 0:
        return np.array([])

    steps = len(img) // batch_size + 1
    img_gen = chunks(img, batch_size, preprocessing_function = preprocess)
    features = model.predict_generator(img_gen, steps = steps)

    # if the generator has looped past end of array, cut it down
    features = features[:len(img)]

    # flatten last three dimension to one
    features = features.reshape(features.shape[0], np.prod(features.shape[1:]))
    return features


def chunks(l, n, preprocessing_function = None):
    '''Yield successive n-sized chunks from l.'''

    func = (lambda x: x) if (preprocessing_function is None) else preprocessing_function

    # in predict_generator, steps argument sets how many times looped through 'while True'
    while True:
        for i in range(0, len(l), n):
            yield np.array([func(d) for d in l[i:i+n]])


def load_features(model_name):
    '''Load features.'''
    start = time.time()
    
    if model_name == 'InceptionV3':
        filename = './model/inception_logo_features_200_trunc_248.hdf5'
    elif model_name == 'VGG16':
        filename = './model/vgg16_logo_features_128.hdf5'
        
    # get database features
    with  h5py.File(filename, 'r') as hf:
        brand_map = list(hf.get('brand_map'))
        input_shape = list(hf.get('input_shape'))
        features = hf.get('features')
        features = np.array(features)
    
    print('Loaded {} features from {} in {:.2f}sec'.format(features.shape, filename, time.time()-start))

    return features, brand_map, input_shape


def save_features(filename, features, brand_map, input_shape):
    '''Save features to compressed HDF5 file.'''
    # reduce file size by saving as float16
    features = features.astype(np.float16)
    
    start = time.time()
    with h5py.File(filename, 'w') as hf:
        hf.create_dataset('features', data = features, compression='lzf')
        hf.create_dataset('brand_map', data = brand_map)
        hf.create_dataset('input_shape', data = input_shape)

    print('Saving {} features into {} in {:.2f} secs'.format(features.shape, filename, time.time() - start))
 

def load_extractor_model(model_name):
    '''Load variant of specified model.'''
    
    start = time.time()
    if model_name == 'InceptionV3':
        from keras.applications.inception_v3 import InceptionV3
        from keras.applications.inception_v3 import preprocess_input
        model = InceptionV3(weights='imagenet', include_top=False)

        trunc_layer = [-1, 279, 248, 228, -1]
        i_layer = 2
        model_out = Model(inputs=model.inputs, 
                          outputs=model.layers[trunc_layer[i_layer]].output)
        input_shape = (200, 200, 3) #(299,299,3) if flavor==0 else (200,200,3)

    elif model_name == 'VGG16':
        from keras.applications.vgg16 import VGG16
        from keras.applications.vgg16 import preprocess_input
        model_out = VGG16(weights='imagenet', include_top=False)
        input_length = 128 #[224,128,64][flavor]
        input_shape = (input_length,input_length,3)

    print('Loaded {} feature extractor in {:.2f}sec'.format(model_name, time.time()-start))
    
    return model_out, preprocess_input, input_shape
    

def construct_DB(DB_list, model_name, DB_path):
    '''Consturct the database of features from img_path.'''
    
    start = time.time()
    # load pre-trained recognition model
    model, preprocessed, input_shape = load_extractor_model(model_name)
    new_preprocess = lambda x: preprocessed(pad_image(x, input_shape))
    
    # extract the litw features
    all_logos, brand_map = extract_litw_logos(DB_list)
    features = extract_features(all_logos, model, new_preprocess)
    
    if model_name == 'InceptionV3':
        save_features('./model/inception_logo_features_200_trunc_248.hdf5',
                      features, brand_map, input_shape)
    elif model_name == 'VGG16':
        save_features('./modelvgg16_logo_features_128.hdf5',
                      features, brand_map, input_shape)
    print('Elapsed Time: {:.2f}'.format((time.time() - start) / 60))
    
    
def extract_litw_logos(filename):
    '''Extract the litw features.'''
    
    with open(filename, 'r') as file:
        img_list = []
        bbox_list = []
        for line in file.read().splitlines():
            img, bbox = line.split(' ')[0],  line.split(' ')[1:]
            img_list.append(img)

            bbox = [ bb for bb in bbox if bb != '' ]

            # skip if no predictions made
            if len(bbox)==0:
                bbox_list.append([])
                continue

            if len(bbox[0].split(','))==5:
                bbox = [[int(x) for x in bb.split(',')] for bb in bbox]
            elif len(bbox[0].split(','))==6:
                bbox = [[int(x) for x in bb.split(',')[:-1]] + [float(bb.split(',')[-1])] for bb in bbox]
            else:
                print(bbox[0])

            # sort objects by prediction confidence
            bbox = sorted(bbox, key = lambda x: x[-1], reverse=True)
            bbox_list.append(bbox)
            
    all_logos = []
    brand_map = []
    for idx in range(len(bbox_list)):
        img = cv2.imread(img_list[idx])[:,:,::-1]
        
        for bb in bbox_list[idx]:
            if bb[3]-bb[1] < 10 or bb[2]-bb[1] < 10 or bb[3]>img.shape[0] or bb[2]> img.shape[0]:
                continue
            all_logos.append(img[bb[1]:bb[3], bb[0]:bb[2]])
            brand_map.append(bb[-1])

    return all_logos, brand_map

def similarity_cutoff(feat_input, features, threshold):
    """
    Given list of input feature and feature database, compute distribution of
    cosine similarityof the database with respect to each input. Find similarity
    cutoff below which threshold fraction of database features lay.
    """

    start = time.time()
    cs = cosine_similarity(X = feat_input, Y = features)

    cutoff_list = []
    cdf_list = []
    for i, cs1 in enumerate(cs):
        hist, bins = np.histogram(cs1, bins=np.arange(0,1,0.001))
        cdf = np.cumsum(hist)/len(cs1)
        cutoff = bins[np.where(cdf < threshold)][-1]
        cutoff_list.append(cutoff)
        cdf_list.append(cdf)
    end = time.time()
    print('Computed similarity cutoffs given inputs in {:.2f}sec'.format(end - start))

    return cutoff_list, (bins, cdf_list)