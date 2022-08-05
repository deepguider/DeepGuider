import os
import time
import glob
from ocr_recognizer import OCRRecognizer
from tqdm import tqdm

imagepath = '/gallery_moma/minjung.kim/project/coex_demo/220418_uvc_image' #'demo_image'

start = time.time()

test = OCRRecognizer()
test.initialize()

imgs = glob.glob(os.path.join(imagepath, '*.jpg'))
imgs.sort()

with open('pred_list_uvc2.txt', 'w') as f:
    for img in tqdm(imgs):
        pred, timestamp = test.apply(img, start, True)
        for p in pred:
            f.write(p[1]+' ')
        #base = os.path.basename(img)
        f.write(img+'\n')
        #captured = float(base.split('_')[1].split('.')[0]) / 60
        #f.write(base + ' ' + f'{captured:.2f}\n')
    
print('\nDone! It tooks {:.2f} sec.\n'.format((time.time() - start)))
