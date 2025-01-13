#!/bin/bash


echo "connecting can0"

sudo modprobe can
sudo modprobe kvaser_usb
sudo ifconfig can0 down
sudo ip link set can0 type can bitrate 500000

sudo ifconfig can0 up

cansend can0 005A6401#5A01000000000000
