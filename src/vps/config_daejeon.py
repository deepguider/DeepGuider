import os

#root_dir = "./data_vps/dataset/ImageRetrievalDB/custom_dataset_daejeon_dbNaver_qPhone"
root_dir = "./data_vps/netvlad_etri_datasets"
db_dir = os.path.join(root_dir, '.')
queries_dir = os.path.join(root_dir, '.')


if not os.path.exists(root_dir) or not(db_dir):
    raise FileNotFoundError("root_dir : {}, db_dir : {}".format(root_dir, db_dir))

struct_dir = os.path.join(root_dir, 'datasets') # For mat files in which list of image files are

#structFile = join(struct_dir, 'pitts30k_test.mat')
structFile = os.path.join(struct_dir, 'dg_daejeon_test.mat')
#structFile = os.path.join(struct_dir, 'dg_seoul_test.mat')

x, y, coord, radius = 36.3845254,127.3698013, 'latlon', 50 
#x, y, coord, radius = 36.38689038675593, 127.37838523982111, 'latlon', 25
#x, y, coord, radius = 354559.16244696, 4028082.80969047, 'utm', 50
