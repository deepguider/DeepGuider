
def makedir(fdir):
    import os
    if not os.path.exists(fdir):
        os.makedirs(fdir)

def filecopy(srcdir,dstdir,fnwild):
    """
from myutils.file_utils import filecopy
filecopy('src','dst','*.jpg')
    """
    import os
    import glob
    import shutil
    from ipdb import set_trace as BP

    rootdir=os.path.join(os.getcwd(),srcdir)
    tfcnt=0
    subdir=os.listdir(srcdir)
    subdir.append('.')
    
    for sdir in subdir:
        sdirpath=os.path.join(rootdir,sdir)
        if not os.path.isdir(sdirpath):
            continue
        InputFileList=glob.glob(os.path.join(sdirpath,fnwild))
        tfcnt += len(InputFileList)
    
        for i,fn in enumerate(InputFileList):
            fname=fn.split('/')[-1]
            fext=fname.split('.')[-1]
            newfname='{:08d}.{}'.format(i,fext)
            dstfn=os.path.join(dstdir,newfname)
            shutil.copy(fn,dstfn)
    
    print("Total {} files were copied to {}".format(tfcnt,dstdir))
