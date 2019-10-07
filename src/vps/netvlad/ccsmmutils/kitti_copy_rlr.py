import os
import glob
import shutil
import numpy as np
from ipdb import set_trace as BP
from myutils import file_utils as fu

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

copied = 0
fu.makedir(dstdir)
traintxt = open(os.path.join(dstdir,'train.txt'), 'w')
valtxt = open(os.path.join(dstdir,'val.txt'), 'w')


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
#    InputFileListL=np.random.permutation(InputFileListL)


    for i,fnl in enumerate(InputFileListL):
        fnr=fnl.replace(lstr,rstr)
        if (not os.path.exists(fnl)) or (not os.path.exists(fnr)):
            continue

        copied += 1
        
        fname=fnl.split('/')[-1]
        fext=fname.split('.')[-1]
        newfn0r='{:08d}_0r_{}.{}'.format(i,rstr,fext)
        newfn1l='{:08d}_1l_{}.{}'.format(i,lstr,fext)
        newfn2r='{:08d}_2r_{}.{}'.format(i,rstr,fext)

        eachdir='RLR{:08d}'.format(copied) #Right/Left/Right

        dstfn=os.path.join(dstdir,eachdir)
        fu.makedir(dstfn)

        if (i+1)%10 == 0:
            valtxt.write(eachdir+'\n')
        else:
            traintxt.write(eachdir+'\n')

        dstfn0r=os.path.join(dstfn,newfn0r)
        dstfn1l=os.path.join(dstfn,newfn1l)
        dstfn2r=os.path.join(dstfn,newfn2r)

        shutil.copy(fnr,dstfn0r)
        shutil.copy(fnl,dstfn1l)
        shutil.copy(fnr,dstfn2r)


        TxtFileList=glob.glob(os.path.join(lpath,'*.txt'))
        for fn in TxtFileList:
            shutil.copy(fn,dstfn)

    tfcnt += copied


traintxt.close()
valtxt.close()
print("Total {} files were copied to {}".format(tfcnt,dstdir))
