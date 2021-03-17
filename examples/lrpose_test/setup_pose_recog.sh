#!/bin/bash
##Current Directory : topdir/examples/intersection_test

PWD=`pwd`
cd ../../bin
ln -sf ../src/lrpose_recog/data_lrpose
if [ ! -f "ckpt.pth.best.acc96p" ]; then
	cat data_lrpose/pretrained_url.md
fi
cd $PWD   # return to initial directory
