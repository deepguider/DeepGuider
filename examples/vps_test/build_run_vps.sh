#!/bin/bash

function python_venv() {
	local topdir=$1
	local CWD=`pwd`
	#local venvbin=/home/you/.virtualenvs/dg_venv3.6/bin
	local venvbin=`cat $topdir/bin/venvbin.txt 2> /dev/null`
	echo $venvbin
	if [ ! -z $venvbin ];then # Not null,
		if [ -d $venvbin ];then # If existed,
			source $venvbin/activate
			#exit 1
		else
			echo "[XXX] Not found : $venvbin/activate"
		fi
	else
		echo "[XXX] You need to make venvbin.txt in $topdir/bin with a line as follows with no spaces:"
		echo "/home/YourID/.virtualenvs/dg_venv3.6/bin"
	fi
	cd $CWD
}


function check_server() {
	local topdir=$1
	local CWD=`pwd`
	python_venv $topdir
	echo "#### Check if server_vps.py is running  ####"
	isrunning=`ps -ef | grep server_vps.py | grep -v grep | grep -v bash`
	if [ -z "$isrunning" ];then
		cd $topdir/src/vps
		gnome-terminal -x bash -c "python server_vps.py"
		isrunning=`ps -ef | grep server_vps.py | grep -v grep | grep -v bash`
		if [ -z "$isrunning" ];then
			echo "[XXX] Server_vps cannot start."
		else
			echo "[OOO] Server_vps started."
		fi
	else
		echo "[OOO] Server_vps is already running."
	fi
	cd $CWD
}

##Current Directory : topdir/examples/vps_test
CWD=`pwd`
cd ../../
topdir=`pwd`
cd $CWD

echo "####### Compile ..... #######"
cd $topdir/examples/vps_test
mkdir -p build
cd build
cmake ..
make install

if [ $? -ne 0 ];then # If it met error in previous step
	#echo $?
	exit 0
fi

echo "####### Run  ..... #######"
cd $topdir/bin
ln -sf ../src/vps/data_vps

hostname |grep ccsmm 1>/dev/null && ln -sf ~/Naverlabs/query_etri_cart/191115_ETRI.avi data/ || echo "You need to put 19115_ETRI.avi at $topdir/bin/data/."

python_venv $topdir
check_server $topdir
cd $topdir/bin
#gdb ./vps_test
#kdbg ./vps_test
./vps_test
