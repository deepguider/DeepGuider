## Door Detection

### Getting started
Create a virtual environment to run and install the necessary modules.
```
conda create --name door_detect python=3.7
conda activate door_detect
pip install -r requirements.txt
```

### For training
Put the dataset into the `dataset/images` and `dataset/labels` folders.
```
python train.py --img 640 --batch 16 --epochs 3 --data door.yaml --weights checkpoints/best_s.pt
```

### For testing
Put input images in `./test`.
```
python test_wrapper.py
```