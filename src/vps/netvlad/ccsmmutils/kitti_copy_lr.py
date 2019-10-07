import os
import glob
import shutil
from ipdb import set_trace as BP

#rootdir='./'
srcdir='src'
dstdir='dst'
ftype="*.jpg"
lstr='sync_02'
rstr='sync_03'

rootdir=os.path.join(os.getcwd(),srcdir)
tfcnt=0
subdir=os.listdir(srcdir)
subdir.append('.')

for sdir in subdir:
    sdirpath=os.path.join(rootdir,sdir)
    if not os.path.isdir(sdirpath):
        continue

    if ('sync_02' not in sdir):
        continue
    lpath=sdirpath
#    rpath=lpath.replace('sync_02','sync_03')
    InputFileListL=glob.glob(os.path.join(lpath,ftype))
#    InputFileListR=glob.glob(os.path.join(rpath,ftype))

    InputFileListL.sort()

    copied = 0

    for i,fnl in enumerate(InputFileListL):
        fnr=fnl.replace(lstr,rstr)
        if (not os.path.exists(fnl)) or (not os.path.exists(fnr)):
            continue

        copied += 1
        fname=fnl.split('/')[-1]
        fext=fname.split('.')[-1]
        newfnl='{:08d}_{}.{}'.format(i,lstr,fext)
        newfnr='{:08d}_{}.{}'.format(i,rstr,fext)

        dstfnl=os.path.join(dstdir,newfnl)
        dstfnr=os.path.join(dstdir,newfnr)
        shutil.copy(fnl,dstfnl)
        shutil.copy(fnr,dstfnr)


    TxtFileList=glob.glob(os.path.join(lpath,'*.txt'))
    for fn in TxtFileList:
        shutil.copy(fn,dstdir)

    tfcnt += copied

print("Total {} files were copied to {}".format(tfcnt,dstdir))
