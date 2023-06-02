#!/bin/bash

# show commands before execution and exit when errors occur
set -e -x

# install required dependencies 
cd /tmp/install/px4_setup
bash ubuntu.sh --no-sim-tools --no-nuttx

# required python dependencies
pip3 install --user -U empy pyros-genmsg setuptools

# install XRCE-DDS Agent
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/