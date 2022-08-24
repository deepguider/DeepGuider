import os
import time
import glob
from ocr_recognizer import OCRRecognizer
from tqdm import tqdm

imagepath = 'demo_image'
language = 'kr'
if language == 'kr':
    saved_model = './data_ocr/best_accuracy_kr.pth'
else: # language == 'en'
    saved_model = './data_ocr/best_accuracy_en.pth'

start = time.time()

test = OCRRecognizer(language=language)
test.initialize(saved_model=saved_model)

imgs = glob.glob(os.path.join(imagepath, '*.png'))
imgs.sort()

with open('pred_list.txt', 'w') as f:
    for img in tqdm(imgs):
        pred, timestamp = test.apply(img, start)
        for p in pred:
            f.write(p[1]+' ')
        #base = os.path.basename(img)
        f.write(img+'\n')
        #captured = float(base.split('_')[1].split('.')[0]) / 60
        #f.write(base + ' ' + f'{captured:.2f}\n')
    
print('\nDone! It tooks {:.2f} sec.\n'.format((time.time() - start)))
