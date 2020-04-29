## OCR Recognition

We train text recognition(OCR) classifier based on [deep-text-recognition-benchmark](https://github.com/clovaai/deep-text-recognition-benchmark)

## Getting Started


### Dependency
- Expriments were performed with **python 3.6, PyTorch 0.4.1, CUDA 9.0**.
- requirements : lmdb, pillow, torchvision, nltk, natsort

### Download dataset for traininig and evaluation from [here](https://drive.google.com/open?id=1hywOh26U5BrX6a9depZ2JbEsUktlDbbM) 
data_lmdb_releas.zip contains below. <br>
training datasets : [MJSynth (MJ)](http://www.robots.ox.ac.uk/~vgg/data/text/), [SynthText (ST)](http://www.robots.ox.ac.uk/~vgg/data/scenetext/) and the union of the training sets [IC13](http://rrc.cvc.uab.es/?ch=2), [IC15](http://rrc.cvc.uab.es/?ch=4), [IIIT](http://cvit.iiit.ac.in/projects/SceneTextUnderstanding/IIIT5K.html), and [SVT](http://www.iapr-tc11.org/mediawiki/index.php/The_Street_View_Text_Dataset).\
validation/evaluation datasets : benchmark evaluation datasets, [IC13](http://rrc.cvc.uab.es/?ch=2)


### For training and testing

1. Train Model
```
python train.py --experiment_name ocr_train \
--train_data data_lmdb_release/training --valid_data data_lmdb_release/validation \
--select_data MJ-ST-b_DataSet \
--batch_ratio 0.45-0.45-0.1 --batch_size 240 
```

 2. Test our best accuracy model. Download pretrained model from [here](https://drive.google.com/open?id=1FO_lt3US-gc8xwK97iE8R4on_iuFzaan) and put it into `checkpoints/`.
```
python test.py --eval_data data_lmdb_release/evaluation --benchmark_all_eval --saved_model checkpoints/best_accuracy.pth
```

### Performance 

Model |  IC13_857 | IC13_1015 | avg 
-- | -- | -- | -- | 
Our best model   | 94.866 | 92.906 | 93.80 
