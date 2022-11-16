#!/bin/bash

function get_prebuilt_dir(){
    ## Usage : get_prebuilt_dir;selected_dir=$ret
	prebuilt_dirlist=`ls -1d prebuilt_dbfeat_custom_dataset*`
    echo;
    cnt=0
    for d in $prebuilt_dirlist
    do  
        cnt=`expr $cnt + 1`
        echo "[$cnt] $d"
    done
	echo "-------------------------------------"
    echo "Choose one number among above dataset"
    read KeyPress
    case "$KeyPress" in
        [0-9]   ) echo "You have choose $KeyPress";;
        *       )
    esac
    cnt=0
    for d in $prebuilt_dirlist
    do  
        cnt=`expr $cnt + 1`
        [ "$cnt" -eq "$KeyPress" ] && ret=$d
    done
    #echo $ret
}

function link_src_to_catkin(){
	if [ ! "${1}" ]; then
	    ROS_WS_DIR="catkin_ws"
	else
	    ROS_WS_DIR="${1}"
	fi

    CWD=`pwd`
    User=`whoami`  # ccsmm
    cd ${CWD}
    
    SRC_TOP="$CWD"
    DST_TOP="/home/${User}/${ROS_WS_DIR}/data_vps"
    
    DB_SRC="${SRC_TOP}/dbImg/StreetView"
    DB_DST="${DST_TOP}/netvlad_etri_datasets/dbImg"
    
    FEAT_SRC="${SRC_TOP}/prebuilt_dbFeat.pickle"
    FEAT_DST="${DST_TOP}"
    
    if [ -e ${DB_DST} ]; then
    	echo ""
    	echo ">>> Flushing and symbolic linking streetview images and prebuilt features to ${DB_DST}"
    	rm -rf ${DB_DST}/*
    	echo "    > ln -s ${DB_SRC} to ${DB_DST}"
    	echo "    > ln -s ${FEAT_SRC} to ${FEAT_DST}"
    	mkdir -p ${DB_DST}
        ln -sf ${DB_SRC}/ ${DB_DST}/
        ln -sf ${FEAT_SRC} ${FEAT_DST}/.
    
    	echo ""
    	echo "If you got ramdisk mount error, re-run this after running ${ROS_WS_DIR}/init_vps_ramdisk.sh"
    	
    	## Misc.
    		echo "###### Be careful to make symbolic link including large amount files in ros workspace,"
    		echo "  because ros takes times to search and index all files in package workspace at starting time. ######"
    
    	## Check error on return value of above commands
    	if [ $? -eq 1 ]; then # Error
    		echo "Some error have occured. Did you run ./setup_all.sh in examples directory before running ${0}."
    		echo "Run if you do not have dg_seoul_test.mat in dg_bin/data_vps/dataset/ImageRetrievalDB/custom_dataset_seoul_dbRobot_qRobot_220418/datasets/"
    		echo "Run git clone github.com/ccsmm78/dataset.git at ~/dg_bin/data_vps/"
    	fi
    else
    	echo ">>>No target directory exists : ${DB_DST}"
		mkdir -p ${DB_DST};echo "I made ${DB_DST}, You can Re-run $0"
		echo "Usage :"
		echo "       $0  # means default : $0 catkin_ws"
		echo "       $0 catkin_ws"
		echo "       $0 ros_deepguier_ws"
    fi
}

if [ ! "${1}" ]; then
    ROS_WS_DIR="catkin_ws"
else
    ROS_WS_DIR="${1}"
fi

cd data_vps
get_prebuilt_dir # return $ret
cd $ret
link_src_to_catkin $ROS_WS_DIR
