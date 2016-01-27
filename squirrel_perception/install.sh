#!/bin/sh
sudo apt-get install dpkg
sudo dpkg -i ros-*v4r*.deb
sudo apt-get install -f
