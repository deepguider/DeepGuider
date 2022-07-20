## OCR Recognition

We modified and trained text recognition (OCR) classifier based on [CRAFT](https://github.com/clovaai/CRAFT-pytorch) and [deep-text-recognition-benchmark](https://github.com/clovaai/deep-text-recognition-benchmark).

## Getting Started


### Dependency
- Expriments were performed with **python 3.8, PyTorch 1.9.0, torchvision 0.10.0, CUDA 10.1**.
- Requirements : lmdb, pillow, torchvision, nltk, natsort, scikit-learn, opencv-python

### Download dataset for traininig and evaluation from [here](https://drive.google.com/open?id=1hywOh26U5BrX6a9depZ2JbEsUktlDbbM).
data_lmdb_release.zip contains below. <br>
training datasets : [MJSynth (MJ)](http://www.robots.ox.ac.uk/~vgg/data/text/), [SynthText (ST)](http://www.robots.ox.ac.uk/~vgg/data/scenetext/), SynthText (Kr) from [IC19](https://rrc.cvc.uab.es/?ch=15&com=tasks) and the union of the training sets [IC13](http://rrc.cvc.uab.es/?ch=2), [IC15](http://rrc.cvc.uab.es/?ch=4), [IIIT](http://cvit.iiit.ac.in/projects/SceneTextUnderstanding/IIIT5K.html), [SVT](http://www.iapr-tc11.org/mediawiki/index.php/The_Street_View_Text_Dataset) and Korean from [IC19](https://rrc.cvc.uab.es/?ch=15&com=tasks).\
validation/evaluation datasets : benchmark evaluation datasets, [IC13](http://rrc.cvc.uab.es/?ch=2) and Korean from [IC19](https://rrc.cvc.uab.es/?ch=15&com=tasks).


### For training and testing

1. Train model.
```
python train.py --experiment_name ocr_train \
--train_data data_lmdb_release/training --valid_data data_lmdb_release/validation \
--select_data MJ-ST-b_DataSet-kr_synth-train_kr --batch_ratio 0.2-0.2-0.1-0.4-0.1 --FT \
--workers 0 --use_tb --batch_size 240 --saved_model checkpoints/best_accuracy.pth
```

 2. Test our best accuracy model. Download pretrained model from [recognition](https://drive.google.com/file/d/1hTjJDJNY98CZRrADNpCOOVZsbZwBE4zC/view?usp=sharing) and put it into `checkpoints/`.
```
python test.py --eval_data data_lmdb_release/evaluation --benchmark_all_eval --saved_model checkpoints/best_accuracy.pth
```

### Run demo
1. Download pretrained models from [here](https://drive.google.com/drive/folders/12b60GO8rAMXV5-bbXJyR4x5hYwwqltmZ). <!--[detection](https://drive.google.com/file/d/1R3hzHWwDJ_cKp__KcIvDg4V1hikT9MvU/view?usp=sharing), [recognition](https://drive.google.com/file/d/1hTjJDJNY98CZRrADNpCOOVZsbZwBE4zC/view?usp=sharing) -->
3. Set image path and run test_wrapper.py
4. Check ./data_ocr/result.jpg
```
python test_wrapper.py
```


### Performance

Model |  IC13_857 | IC13_1015 | avg
-- | -- | -- | -- |
Our best model   | 96.033 | 94.975 | 95.459
