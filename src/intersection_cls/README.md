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
    
- Download weight (v5.0 indoor and outdoor) file from this 
[link](https://drive.google.com/file/d/1rPrYu3ReLvHEz9yXqGJlcrHnEWKsOdWm/view?usp=sharing) 
, put it inside *data_intersection_cls* folder and rename it as *weight.pth*

Other models
- v5.0 (outdoor): [link](https://drive.google.com/file/d/12aUd0IoZ6k4IYra1TslxYAoiwWBgu6xs/view?usp=sharing)
- v5.0 (indoor): [link](https://drive.google.com/file/d/1gOsqg19xNV7o51jF8OJ1cN3E7pZrwpFf/view?usp=sharing)
- v1.10 (outdoor): [link](https://drive.google.com/file/d/18arE8seqD4xnhYSY8oVDtBh4-07WeH0I/view?usp=sharing) 
- v1.1 (outdoor): [link](https://drive.google.com/file/d/11lwdIjx-GEyzVRa4W0whZ9y-ZdG_ZRE2/view?usp=sharing) 
- v0.14 (outdoor): [link](https://drive.google.com/file/d/11iMMgd3dq66N3nZ3-2GLD8K8gA5RM2oM/view?usp=sharing)
    
### Run demo
- Run demo from this current folder `python intersection_cls.py`
- Output should be 
```
processing time: 0.0266 seconds
nonintersection_img demo: class  0  confidence  1.0
processing time: 0.0205 seconds
intersection_img demo: class  0  confidence  0.9999996423721313
processing time: 0.0196 seconds
nonintersection_img2 demo: class  0  confidence  0.9999991655349731
processing time: 0.0406 seconds
nonintersection_img3 demo: class  1  confidence  0.9776144027709961
```
- Output of older models (v1.10)
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

### Outputs
- Class
    - predicted class described with value {0, 1}. 0: non-intersection class. 1: intersection class
- confidence
    - classification confidence (range 0-1 with 1 the highest confidence)


    
