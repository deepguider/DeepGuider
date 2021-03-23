#!/bin/bash
##Current Directory : topdir/examples/.vscode_DoNotEdit

CWD=`pwd`
cd ../../
topdir=`pwd`
cd $CWD

mkdir -p ../.vscode
cp ./*.json ../.vscode/.

sed -e "s|/work/deepguider|${topdir}|g" c_cpp_properties.json > ../.vscode/c_cpp_properties.json

cd $CWD   # return to initial directory
