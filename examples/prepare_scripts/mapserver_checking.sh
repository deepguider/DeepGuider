#!/bin/bash
server_ip="localhost"
#server_ip="129.254.81.204"
test_dir="./.test_img_server"
mkdir -p ${test_dir}
cd ${test_dir}
rm -rf *.jpg

curl http://${server_ip}:10000/64141101818/f --output temp10000_etri.jpg
curl http://${server_ip}:10001/64193400869/f --output temp10001_bongeunsa.jpg
curl http://${server_ip}:10002/90636102228/f --output temp10002_technopark.jpg
curl http://${server_ip}:10003/1621318282779869/0 --output temp10003_etri_indoor.jpg

chksum_true="6e294aa72fd0b73aa86201aae6cd818a"
chksum=`find . -type f -exec md5sum {} \; | sort -k 2 | cut -f1 -d' ' | md5sum | cut -f1 -d' '`

echo ""
echo "###########################################################"
if [ $chksum_true == $chksum ]; then
	echo "[chksum passed] Map server(${server_ip}) is running."
else
	echo "[chksum failed] Map server(${server_ip}) has error."
fi

geeqie

cd ..
rm -rf ${test_dir}
