## Prerequisites
- Tested on: CUDA 10.0
- Python 3.6
    - It's recommended to use a virtualenv, if possible. What I use:
        - pyenv: https://github.com/pyenv/pyenv
        - pyenv-virtualenv: https://github.com/pyenv/pyenv-virtualenv
        - After installing both pyenv and pyenv-virtualenv, run
        ```
        pyenv install 3.6.5  # install python 3.6.5
        pyenv virtualenv 3.6.5 intersectioncls  # create virtual environment with name intersectioncls
        pyenv activate intersectioncls  # enter the virtual environment
        ```
- Install dependencies `pip install -r requirement_intersection_cls.txt`. Consists of:
    - Numpy 1.18.1
    - OpenCV 4.1.1.26
    - Pillow 6.1.0
    - PyTorch 1.2.0
    - Torchvision 0.4.0
    - Dropblock 0.3.0
    
- Download weight file from this 
[link](https://drive.google.com/file/d/1gA2z28QPA0W0UbtC7J2kX3QXLOYCKIvz/view?usp=sharing) 
, put it inside *data_intersection_cls* folder and rename it as *weight.pth*

Older models
- v1: [link](https://drive.google.com/file/d/15zZC16vN7oavhX-dx3oAth39loJp31Ay/view?usp=sharing) 
- v0: [link](https://drive.google.com/file/d/1sX411P95LHE6kB7yzgmHq537uvZRUcA-/view?usp=sharing)
    
### Run demo
- Run demo from this current folder `python intersection_cls.py`
- Output should be 
```
processing time: 0.0557 seconds
nonintersection_img demo: class  0  confidence  0.9951779842376709
processing time: 0.0392 seconds
intersection_img demo: class  1  confidence  0.9140893220901489
processing time: 0.0317 seconds
nonintersection_img2 demo: class  0  confidence  0.9989577531814575
processing time: 0.0603 seconds
nonintersection_img3 demo: class  0  confidence  0.9921876192092896
```
- Output of older models (v1)
```
processing time: 0.0526 seconds
nonintersection_img demo: class  0  confidence  0.9880973696708679
processing time: 0.0377 seconds
intersection_img demo: class  1  confidence  0.9951642751693726
processing time: 0.0314 seconds
nonintersection_img2 demo: class  0  confidence  0.8869904279708862
processing time: 0.0604 seconds
nonintersection_img3 demo: class  1  confidence  0.5098475217819214
```

### Outputs
- Class
    - predicted class described with value {0, 1}. 0: non-intersection class. 1: intersection class
- confidence
    - classification confidence (range 0-1 with 1 the highest confidence)


    