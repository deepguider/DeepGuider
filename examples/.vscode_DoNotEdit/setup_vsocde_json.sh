#!/bin/bash
##Current Directory : topdir/examples/.vscode_DoNotEdit

CWD=`pwd`
cd ../../
topdir=`pwd`
cd $CWD

python_include=`python3 -c "from sysconfig import get_paths as gp; print(gp()['include'])"`

mkdir -p ../.vscode
cp ./*.json ../.vscode/.

if [ -e ${topdir} ]; then
	sed -i "s|/work/deepguider|${topdir}|g" ../.vscode/c_cpp_properties.json
fi

if [ -e ${python_include} ]; then
	sed -i "s|/usr/include/python3.6m|${python_include}|g" ../.vscode/c_cpp_properties.json
fi

cd $CWD   # return to initial directory
