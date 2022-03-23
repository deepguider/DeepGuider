import os

root_dir = './data_vps/custom_dataset/dataset_seoul'
db_dir = os.path.join(root_dir, '.')
queries_dir = os.path.join(root_dir, '.')


if not os.path.exists(root_dir) or not(db_dir):
    raise FileNotFoundError("root_dir : {}, db_dir : {}".format(root_dir, db_dir))

struct_dir = os.path.join(root_dir, 'datasets') # For mat files in which list of image files are

#structFile = join(struct_dir, 'pitts30k_test.mat')
#structFile = os.path.join(struct_dir, 'dg_daejeon_test.mat')
structFile = os.path.join(struct_dir, 'dg_seoul_test.mat')

#x, y, coord, radius = 327934.67464998, 4153535.06119226, 'utm', 25
x, y, coord, radius = 37.511634, 127.061298, 'latlon', 25
