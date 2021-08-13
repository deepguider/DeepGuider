dataset_region="ETRI"
date_idx = 0
begin_skip_frame = 0
ip = "129.254.81.204"
if dataset_region == "ETRI":
    indir = "/home/ccsmm/data1/workdir/DB_Repo2/DeepGuider_system_testset/ETRI"
    input_reference_ext = "*.avi"
elif dataset_region == "COEX":
    indir = "/home/ccsmm/data1/workdir/DB_Repo2/DeepGuider_system_testset/COEX"
    input_reference_ext = "*.mkv"
else:
    indir = "/home/ccsmm/data1/workdir/DB_Repo2/DeepGuider_system_testset/ETRI"
    input_reference_ext = "*.avi"
out_postfix = "vps.csv"
which_gpu = 1
