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
    
- Download weight file from this. (Currently by default, using outdoor model)
outdoor model: [link](https://drive.google.com/file/d/1gA2z28QPA0W0UbtC7J2kX3QXLOYCKIvz/view?usp=sharing) 
indoor models: [link](https://drive.google.com/drive/folders/1bTBlOEDoHgPm0ot3hyOXwgtK1lV2Mo-w?usp=sharing) extract all weights into one folder "v1indoor_resnet18_3camera"

    
### Run demo
- Run demo from this current folder `python intersection3camera_cls.py`
- Output should be 
```
processing time: 1.2924 seconds
nonintersection_img demo: class  0  confidence  0.5133477449417114
processing time: 1.0733 seconds
intersection_img demo: class  1  confidence  0.9986504912376404
```

### Outputs
- Class
    - predicted class described with value {0, 1}. 0: non-intersection class. 1: intersection class
- confidence
    - classification confidence (range 0-1 with 1 the highest confidence)


    
