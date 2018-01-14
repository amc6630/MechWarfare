#! /bin/sh

#stuffz

./mjpg_streamer -i "./plugins/input_raspicam/input_raspicam.so -fps 25 -x 320 -y 240" -o "./plugins/output_http/output_http.so -w ./www"

