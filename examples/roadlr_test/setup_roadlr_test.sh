#!/bin/bash
##Current Directory : topdir/examples/roadlr_test

PWD=`pwd`
cd ../../bin
ln -sf ../src/roadlr/data_roadlr
if [ ! -f "ckpt.pth.best.acc96p" ]; then
	cat data_roadlr/pretrained_url.md
fi
cd $PWD   # return to initial directory
