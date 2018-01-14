#! /bin/sh

#setup script as sudo
modprobe bcm2835-v4l2
modprobe v4l2_common

#mjpg streamer with settings
./mjpg_streamer -i "./plugins/input_uvc/input_uvc.so -y YUYV -r 160x120 -f 40" -o "./plugins/output_http/output_http.so -w ./www"
