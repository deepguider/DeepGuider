import os
import time
import glob
from ocr_recognizer import OCRRecognizer
from tqdm import tqdm

imagepath = 'demo_image'

start = time.time()

test = OCRRecognizer()
test.initialize()

imgs = glob.glob(os.path.join(imagepath, '*.jpg'))
imgs.sort()

with open('pred_list.txt', 'w') as f:
    for img in tqdm(imgs):
        pred, timestamp = test.apply(img, start, True)
        for p in pred:
            f.write(p[1]+'\n')
    
print('\nDone! It tooks {:.2f} sec.\n'.format((time.time() - start)))
