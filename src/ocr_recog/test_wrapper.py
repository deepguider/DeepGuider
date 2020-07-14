
import time
from ocr_recognizer import OCRRecognizer


imagepath = './demo_image/demo_1.png'
imagepath = './demo_image/demo_2.png'

start = time.time()

test = OCRRecognizer()
test.initialize()
pred, timestamp = test.apply(imagepath,start,True)

print('\nDone! It tooks {:.2f} sec.\n'.format((time.time() - start)))
