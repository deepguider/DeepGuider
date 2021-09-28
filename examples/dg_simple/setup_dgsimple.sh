#!/bin/bash
##Current Directory : topdir/examples/dg_simple

cd ../../bin

ln -sf ../src/vps/data_vps
ln -sf ../src/intersection_cls/data_intersection_cls
ln -sf ../src/logo_recog/logo_data
ln -sf ../src/logo_recog/model
ln -sf ../src/ocr_recog/data_ocr
cp ../examples/dg_simple/dg_simple.yml .

cd ../examples/dg_simple   # return to initial directory
