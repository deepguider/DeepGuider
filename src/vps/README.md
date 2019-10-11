These codes are  based on [NetVLAD: CNN architecture for weakly supervised place recognition](https://github.com/Nanne/pytorch-NetVlad) [1].

We use their open source code as very initial example code for VPS of DeepGuider.

After some step, we could make and release our own VPS code.



### To run vps.py
```
# If it's first time to visit here after a git clone, run setup as follows:
./0setup.sh

# Run vps.py
./1run_vps.py
```
#### To do
* Test vps.py with single image
* Train vps using DB captured near ETRI campus



### To train or test netvlad itself which is a sub-fuction of vps.py
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

[1] Arandjelovic, Relja, Petr Gronat, Akihiko Torii, Tomas Pajdla, and Josef Sivic. "NetVLAD: CNN architecture for weakly supervised place recognition." In Proceedings of the IEEE conference on computer vision and pattern recognition, pp. 5297-5307. 2016.