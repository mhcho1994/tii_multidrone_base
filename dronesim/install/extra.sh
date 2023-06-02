#!/bin/bash

# show commands before execution and exit when errors occur
set -e -x

# update/upgrade packages and install required packages
sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y --no-install-recommends install \
	htop \
	ipe \
	iproute2 \
	lcov \
	menu \
	mesa-utils \
	openbox \
	python3-jinja2 \
	python3-numpy \
	python3-vcstool \
	python3-xdg \
	python3-xmltodict \
	qt5dxcb-plugin \
	screen \
	terminator \
	vim

sudo pip3 install pykwalify