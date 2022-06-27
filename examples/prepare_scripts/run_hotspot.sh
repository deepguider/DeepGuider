#!/bin/bash

# Refer https://computingforgeeks.com/create-wi-fi-hotspot-on-ubuntu-debian-fedora-centos-arch/

IFNAME="wlo1"  # you can find with "$ip link show"
CON_NAME="dg_hotspot"

nmcli con add type wifi ifname $IFNAME con-name $CON_NAME autoconnect yes ssid $CON_NAME
nmcli con modify $CON_NAME 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
nmcli con modify $CON_NAME wifi-sec.key-mgmt wpa-psk
nmcli con modify $CON_NAME wifi-sec.psk "dg12345678"
nmcli con up $CON_NAME

## Verify
#nmcli connection show $CON_NAME
