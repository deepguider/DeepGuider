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
    
- Download 1 camera weight file from this 
[link](https://drive.google.com/file/d/1iNyxXq_Ex01kbCBukpHNo8T_m0gyzXpN/view?usp=sharing) 
, put it inside **data_intersection_cls** folder and rename it as **weight_1camera.pth**

- Download 3 camera weight file from this [link](https://drive.google.com/file/d/1JJyd4ddWpESEcDMgw1thj6TjKZdom53-/view?usp=sharing) to **data_intersection_cls** folder. Rename the weight file to **weight_3camera.pth**.
    
### Run demo
- Run demo from this current folder `python intersection_cls.py`

### Outputs
- Class
    - predicted class described with value {0, 1}. 0: non-intersection class. 1: intersection class
- confidence
    - classification confidence (range 0-1 with 1 the highest confidence)


    
