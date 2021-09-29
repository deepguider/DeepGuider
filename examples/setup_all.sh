#!/bin/bash
#by ccsmm@etri.re.kr

function build_unit(){
	local D=$1
	CURR_DIR=`pwd`
	cd ./$D
	mkdir -p build
	cd build
	cmake ..
	make install
	cd ${CURR_DIR}
}

function build_script(){
	local D=$1
	CURR_DIR=`pwd`
	cd ./$D
	found_script=`ls *build*.sh 2> /dev/null|grep -v 'run'`
	if [ ! -z "$found_script" ];then
		echo "[...] Run $D/$found_script"
		bash ./$found_script
		eval "$2=$found_script" # return value via second args
	else
		echo "[...] Run default build script because there is no build_script in $D"
		eval "$2='-1'" # return value via second args
	fi
	cd ${CURR_DIR}
}

function setup_script(){
	local D=$1
	CURR_DIR=`pwd`
	cd ./$D
	setup_switch=".setup.done"
	found_script=`ls *setup*.sh 2> /dev/null`
	if [ ! -z "$found_script" ];then
		if [ ! -f "./$setup_switch" ];then
			echo "[...] Run $D/$found_script"
			bash ./$found_script
			if [ $? -ne 0 ];then # If it met error in previous step
			    echo $?
			    exit 0
			fi
			touch .setup.done
		fi
	fi
	cd ${CURR_DIR}
}

## Current directory is examples
counter=0
for D in $(find . -mindepth 1 -maxdepth 1 -type d)
do
	counter=`expr $counter + 1`;
	echo ""
	echo "[$counter]################# Enter and Compiling [$D] ####################"
	setup_script "$D"
done
