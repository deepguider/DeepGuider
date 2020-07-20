#!/bin/bash

# function python_venv() {
# 	local topdir=$1
# 	local CWD=`pwd`
# 	#venvbin=/home/you/.virtualenvs/dg_venv3.6/bin
# 	venvbin=`cat $topdir/bin/venvbin.txt 2> /dev/null`
# 	if [ ! -z $venvbin ];then # Not null,
# 		if [ -d $venvbin ];then # If existed,
# 			source $venvbin/activate
# 			#exit 1
# 		else
# 			echo "[XXX] Not found : $venvbin/activate"
# 		fi
# 	else
# 		echo "[XXX] You need to make venvbin.txt in $topdir/bin with a line as follows with no spaces:"
# 		echo "/home/YourID/.virtualenvs/dg_venv3.6/bin"
# 	fi
# 	cd $CWD
# }


# function check_vps_server() {
#     local topdir=$1
#     local CWD=`pwd`
#     echo "#### Check if server_vps.py is running  ####"
#     isrunning=`ps -ef | grep server_vps.py | grep -v grep | grep -v bash`
#     if [ -z "$isrunning" ];then
#         cd $topdir/src/vps
#         gnome-terminal -x bash -c "python server_vps.py"
#         isrunning=`ps -ef | grep server_vps.py | grep -v grep | grep -v bash`
# 		if [ -z "$isrunning" ];then
# 			echo "[XXX] Server_vps cannot start because of some reason."
# 			echo "      Try follows and if there are problems, solve it plz."
# 			echo "      $> cd $topdir/src/vps/"
# 			echo "      $> python server_vps.py"
# 			echo "      The problem is mainly because the virtual environment is not set,"
# 			echo "      or packages such as flask and json are not installed."
# 			echo "      To fix the problem, check bin/venvbin.txt and use pip install"
# 			exit -1
# 		else
# 			echo "[OOO] Server_vps started."
# 		fi
#     else
#         echo "[OOO] Server_vps is already running."
#     fi  
#     cd $CWD
# }


# ##Current Directory : topdir/examples/dg_simple
# CWD=`pwd`
# cd ../../
# topdir=`pwd`
# cd $CWD

# echo "####### Compile ..... #######"
# mkdir -p build
# cd build
# cmake ..
# make install
# if [ $? -ne 0 ];then # If it met error in previous step
#     #echo $?
#     cd ..
#     exit 0
# fi

# echo "####### Run ..... #######"

# python_venv $topdir
# # check_vps_server $topdir # If you use server_vps version, un-remark this line

# cd $topdir/bin
# #gdb ./dg_simple #for debugging, you need to insert '-g' option at CXX_FLAGS in your CMakeLists.txt # in your CMakeLists.txt : set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
# #kdbg ./dg_simple #for debugging in graphic mode
# ./dg_simple # for running without debugging

# cd $topdir/examples/dg_simple   # return to initial directory

##Current Directory : topdir/examples/dg_simple
echo "####### Compile ..... #######"
mkdir -p build
cd build
cmake ..
make install
if [ $? -ne 0 ];then # If it met error in previous step
    #echo $?
    cd ..
    exit 0
fi

echo "####### Run ..... #######"
cd ../../../bin

#gdb ./dg_simple #for debugging, you need to insert '-g' option at CXX_FLAGS in your CMakeLists.txt # in your CMakeLists.txt : set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
#kdbg ./dg_simple #for debugging in graphic mode
./dg_simple # for running without debugging

cd ../examples/dg_simple   # return to initial directory