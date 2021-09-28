#!/bin/bash
#by ccsmm@etri.re.kr

function clean_setup_script(){
	local D=$1
	CURR_DIR=`pwd`
	cd ./$D
	setup_switch=".setup.done"
	rm -rf ${setup_switch}
	cd ${CURR_DIR}
}

## Current directory is examples
counter=0
for D in $(find . -mindepth 1 -maxdepth 1 -type d)
do
	counter=`expr $counter + 1`;
	echo ""
	echo "[$counter]################# Enter and Clean .setup.done [$D] ####################"
	clean_setup_script "$D"
done
