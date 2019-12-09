These codes are  based on [NetVLAD: CNN architecture for weakly supervised place recognition](https://github.com/Nanne/pytorch-NetVlad) [1].

We use their open source code as very initial example code for VPS of DeepGuider.

After some step, we could make and release our own VPS code.

## Prerequisit
- Python 3.5 ~
- torch,torchvision, etc.
- visdom  #  for display result


## Prepare Dataset
You can refer src/vps/Dataset_example directory
#### For database images
'src/vps/Dataset_example/dbImg '  has to jpg images that named as 'xxx_IDnum_xxx.jpg'
'src/vps/Dataset_example/dbImg '  can have sub-directory
#### For query  images
'src/vps/Dataset_example/qImg '  can have sub-directory
'src/vps/Dataset_example/dbImg '  may have jpg images of any name
Query image of vps.apply() function will be save at  'src/vps/Dataset_example/dbImg/999_newquery/newquery.jpg' automatically.


## To run vps.py
#### Run visdom and open browser to watch results
```
# Run visdom
(venv)~/DeepGuider/src/vps$ visdom
	Checking for scripts.
	It's Alive!
	INFO:root:Application Started
	You can navigate to http://localhost:8097
# Open http://localhost:8097 with web browser on your machine
```
#### If it's first time to visit here after a git clone, run setup as follows:
```
./0setup.sh
```
#### Run vps.py
```
./1run_vps.py
```

#### Results
```
QueryImage <=================> predicted dbImage
[Q] 2813x.jpg <==> [Pred] 2813x.jpg [Lat,Lon] = 36.0, 127.0 [*Matched]
[Q] 2813x.jpg <==> [Pred] 2813x.jpg [Lat,Lon] = 36.0, 127.0 [*Matched]
[Q] 6035x.jpg <==> [Pred] 2813x.jpg [Lat,Lon] = 36.0, 127.0 : Query from out world
Accuracy : 2 / 3 = 66.66666666666666 % in 10 DB images
You can investigate the internal data of result here. If you want to exit anyway, press Ctrl-D
```

## To train or test netvlad itself which is a sub-fuction of vps.py
```
cd netvlad

# If it's first time to visit here after a git clone, run setup as follows:
./0setup_env.sh

# If you want to run test with pre-trained weight
./3run_test_pitt256k.sh

# If you want to run train
./1run_train.sh

```



Please refer to [**./netvlad/README.md**](netvlad/README.md) to proceed in details.



# Reference

[1] Arandjelovic, Relja, Petr Gronat, Akihiko Torii, Tomas Pajdla, and Josef Sivic. "NetVLAD: CNN architecture for weakly supervised place recognition." In Proceedings of the IEEE conference on computer vision and pattern recognition, pp. 5297-5307. 2

```

```