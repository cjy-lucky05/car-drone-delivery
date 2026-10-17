#!/bin/bash

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="lidar"' >/etc/udev/rules.d/lidar_cp210x.rules
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2e3c", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="nvilidar"' >/etc/udev/rules.d/nvilidar_usb.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/lidar_340.rules
systemctl daemon-reload
service udev reload
sleep 2
service udev restart
