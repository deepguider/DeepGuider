#!/bin/bash
#ccsmm@etri.re.kr

# Refer https://computingforgeeks.com/create-wi-fi-hotspot-on-ubuntu-debian-fedora-centos-arch/
#       https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=ijoos&logNo=221804461551

IFNAME="wlo1"  # you can find with "$ip link show"
CON_NAME="Hotspot"  # It will be saved in /etc/NetworkManager/system-connections as a file
PW="dgtest1234"
IP="192.168.0.10"

if [ $1 ];then
	if [ "$1" = "on" ];then
		nmcli con up $CON_NAME
		echo "SSID:$CON_NAME, passwd:${PW}, IPaddr:$IP"
		ifconfig | grep $IP
	elif [ "$1" = "off" ];then
		nmcli con down $CON_NAME
	elif [ "$1" = "new" ];then
		# It will be saved in /etc/NetworkManager/system-connections as a ${CON_NAME} file
		nmcli con add type wifi ifname $IFNAME con-name $CON_NAME autoconnect yes ip4 $IP/24 ssid $CON_NAME
		nmcli con modify $CON_NAME 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
		nmcli con modify $CON_NAME wifi-sec.key-mgmt wpa-psk
		nmcli con modify $CON_NAME wifi-sec.psk "$PW"
		nmcli con up $CON_NAME
	else
		echo "Usage: $0 [on/off]"
	fi
else
	echo "Usage: $0 [on/off]"
fi


## List in /etc/NetworkManager/system-connections
#nmcli connection

## Delete $CON_NAME file in /etc/NetworkManager/system-connections
#nmcli connection delete $CON_NAME

## Verify
#nmcli connection show $CON_NAME

