#!/bin/bash

function build_unit(){
	D=$1
	cd ./$D
	mkdir -p build
	cd build
	cmake ..
	make install
	cd ../../
}

function build_script(){
	D=$1
	found_script=`ls *build*.sh 2> /dev/null|grep -v 'run'`
	if [ ! -z "$found_script" ];then
		./$found_script
	fi
	cd ..
}

counter=0
for D in $(find . -mindepth 1 -maxdepth 1 -type d)
do
	counter=`expr $counter + 1`;
	echo "[$counter]################# Enter and Compiling [$D] ####################"
	echo ""
	build_unit "$D"
done
