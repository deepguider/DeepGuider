#!/bin/bash
## current dir : ./bin

## Make and mount ramdisk as a temporary disk
ramdisk="/mnt/ramdisk"

if [ ! -d $ramdisk ];then
	sudo mkdir -p ${ramdisk}
	sudo mount -t tmpfs -o size=4G tmpfs ${ramdisk}
	echo "You can mount ramdisk automatically by adding following line at the end of the /etc/fstab"
	echo "  none /mnt/ramdisk tmpfs defaults,size=4G 0 0"
fi
