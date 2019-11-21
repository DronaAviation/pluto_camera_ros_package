#!/bin/sh

echo 'Installing sdl and ffmpeg libraries'
echo 'You might need to enter your password'

sudo apt-get update

# install dependencies
sudo apt-get install libsdl2-dev -y
sudo apt-get install libavcodec-dev -y
sudo apt-get install libswscale-dev -y
sudo apt-get install libavformat-dev -y
sudo apt-get install libavutil-dev -y
sudo apt-get install libavdevice-dev -y

echo 'Installation Done!!'

