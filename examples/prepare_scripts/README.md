1. Copy scripts here to your own dg_bin directory, and add the path in .bashrc
   Then you can run prepare_deepguider_bin.sh in the parent of DeepGuider to
   setup the environment for running dg_simple/dg_simple_ros

2. Except these scripts, you need to set directory tree in dg_bin as follows which may sometimes be updated without notice:
   dg_bin
       Scripts which was copied here.
       ├── data
       │   ├── COEX
       │   └── ETRI
       ├── data_exp
       │   ├── img_trajectory
       │   │   └── homing
       │   ├── model
       │   │   ├── hfnet
       │   │   │   └── variables
       │   │   └── homing
       │   └── optimal_viewpoint
       ├── data_intersection_cls
       ├── data_logo
       ├── data_ocr
       ├── data_roadlr
       ├── data_vps
       │   ├── custom_dataset
       │   │   ├── dataset_daejeon -> /home/dg/DB_Repo/DaejeonDB/image_server_daejeon
       │   │   └── dataset_seoul -> /home/dg/DB_Repo/DaejeonDB/image_server
       │   ├── netvlad
       │   │   └── pretrained_checkpoint
       │   │       ├── vgg16_netvlad_checkpoint
       │   │       │   └── checkpoints
       │   │       └── vgg16_netvlad_checkpoint_gpu4
       │   │           └── checkpoints
       │   ├── netvlad_etri_datasets_indoor_etri12b_1way
       │   │   └── dbImg
       │   │       └── StreetView
       │   └── netvlad_etri_datasets_indoor_etri12b_3way
       │       └── dbImg
       │           └── StreetView
       ├── font
       ├── recordings
       ├── test_dataset
       │   ├── Bucheon
       │   ├── COEX
       │   └── ETRI
       └── video
