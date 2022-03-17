import os
import time
import glob
from door_detector import DoorDetector

imagepath = 'test'

detector = DoorDetector()
pred, timestamp = detector.apply(imagepath, 'timestamp')
print(pred, timestamp)