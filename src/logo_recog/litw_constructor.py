import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import requests
import shutil

from tqdm import tqdm
from pathlib import Path


data_path = './logo_data/LogosInTheWild-v2'
cat_path = list(Path(os.path.join(data_path, 'data')).iterdir())

for path in tqdm(cat_path):
    if os.path.basename(str(path)) == '0samples':
        continue
    with open(os.path.join(str(path), 'urls.txt'), 'r', errors='replace') as f:
        url_list = f.readlines()
    for tmp in tqdm(url_list):
        [num, url] = list(tmp.split())[:2]
        img_path = os.path.join(str(path), 'img' + num + '.jpg') 
        if os.path.exists(img_path):
            os.remove(img_path)
        try:
            res = requests.get(url, stream=True, headers={"User-Agent":"Mozilla/5.0"})
            if res.status_code == 200:
                with open(img_path, 'wb') as f:
                    f.write(res.content)
                del res
        except:
            continue
