'''
Modified from Logohunter, https://github.com/ilmonteux/logohunter
'''

import cv2
import os
import time
import numpy as np

from PIL import Image
from utils import draw_matches, extract_features
from sklearn.metrics.pairwise import cosine_similarity


def detect_logo_only(img_path, yolo, save_img=False, save_img_path='./data/test', crop=False):
    '''Detect logos from image.'''

    start = time.time()
    try:
        image = Image.open(img_path)
        if image.mode != 'RGB':
            image = image.convert('RGB')
    except:
        return 0, 0, 0
    image_array = np.array(image)

    prediction, new_image = yolo.detect_image(image)
    elapsed_t = time.time() - start
    
    if crop:
        for i, (xmin, ymin, xmax, ymax, *_) in enumerate(prediction):
            if xmin > image_array.shape[0] or ymin > image_array.shape[1]:
                continue
            xmin, ymin = int(xmin//1.), int(ymin//1.)
            xmax, ymax = int(np.round(xmax//1.)), int(np.round(ymax//1.))

            if xmax - xmin > 10 and ymax - ymin > 10:
                new = image_array[ymin:ymax, xmin:xmax]
                new = Image.fromarray(new)
                new.save(os.path.join(save_img_path, 'crop_' + str(i) + '_' + os.path.basename(img_path)))
        
    print("Complete the logo detection: {:.3f} secs".format(elapsed_t))

    if save_img:
        new_image.save(os.path.join(save_img_path, 'logo_only_' + os.path.basename(img_path)))

    return prediction, new_image, elapsed_t


def detect_and_match(img_path, model_preproc, params, save_img=False, save_img_path='./data/test', is_test=False):
    '''Detect and Recognize logos from image.'''

    start = time.time()
    if not is_test:
        try:
            image = Image.open(img_path)
            if image.mode != 'RGB':
                image = image.convert('RGB')
            image_array = np.array(image)
        except:
            return 0, 0, 0
    else:
        image = cv2.cvtColor(img_path, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        image_array = np.array(image)
    
    yolo, model, preprocess = model_preproc
    DB_feat, sim_cutoff, bins, cdf_list, DB_labels = params

    prediction, new_image = yolo.detect_image(image)

    # extract portion of bbox and drop the small candidates
    cand = []
    pred = []
    for i, (xmin, ymin, xmax, ymax, *_) in enumerate(prediction):
        if xmin > image_array.shape[0] or ymin > image_array.shape[1]:
            continue
        xmin, ymin = int(xmin//1.), int(ymin//1.)
        xmax, ymax = int(np.round(xmax//1.)), int(np.round(ymax//1.))

        if xmax - xmin > 10 and ymax - ymin > 10:
            cand.append(image_array[ymin:ymax, xmin:xmax])
            pred.append(prediction[i])
    prediction = pred

    # extract features from candidates
    cand_features = extract_features(cand, model, preprocess)
    matches, cos_sim = similar_matches(DB_feat, cand_features, sim_cutoff, bins, cdf_list)

    match_pred = []
    keys = list(matches.keys())
    for k in keys:
        bbox = prediction[k]
        if bbox[-1] < 0.4: # confidence
            matches.pop(k)
            continue
        label = DB_labels[matches[k][0]]
        pred = [bbox[0], bbox[1], bbox[2], bbox[3], label, bbox[-1]]
        match_pred.append(pred)
        print('Logo #{} - {} {} - classified as {} {:.2f}'.format(k, tuple(bbox[:2]), tuple(bbox[2:4]), label, matches[k][1]))

    elapsed_t = time.time() - start
    print("Complete the logo detection and recognition: {:.3f} secs".format(elapsed_t))

    if save_img:
        new_image = draw_matches(image_array, DB_labels, prediction, matches)
        Image.fromarray(new_image).save(os.path.join(save_img_path, 'test_' + os.path.basename(img_path)))

    return match_pred, new_image, elapsed_t


def similar_matches(DB_feat, cand_feat, sim_cutoff, bins, cdf_list):
    '''Given features of DB images, compute cosine similarity
    and define a match if cosine similarity is above a cutoff.'''

    if len(cand_feat) == 0:
        print('Found 0 logos from 0 classes')
        return {}, np.array([])

    cos_sim = cosine_similarity(X = DB_feat, Y = cand_feat)
    cos_sim = np.round(cos_sim, 3) # approximate cos_sim for consistency

    # for each input, return matches if above threshold
    matches = {}
    for i in range(len(DB_feat)):
        match_indices = np.where(cos_sim[i] >= sim_cutoff[i])
        # to avoid double positives, it will pick feat with better cosine similarity
        for idx in match_indices[0]:
            cdf_match = cdf_list[i][bins[:-1] < cos_sim[i, idx]][-1]
            if idx not in matches: # if cand not seen previously, current brand is best guess
                matches[idx] = (i, cdf_match)
            elif matches[idx][1] < cdf_match: # previously seen but lower confidence, replace with current cand
                matches[idx] = (i, cdf_match)
            else:
                continue

    n_classes = len(np.unique([v[0] for v in matches.values()]))
    print(f'Found {len(matches)} logos form {n_classes} classes.')

    return matches, cos_sim
