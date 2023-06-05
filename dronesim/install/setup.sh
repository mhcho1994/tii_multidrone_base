#!/bin/bash

# show commands before execution and exit when errors occur
set -e -x

# add the line 'source /opt/ros/humble/setup.bash' to .bashrc 
# if not already present
if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "Added 'source /opt/ros/humble/setup.bash' to .bashrc"

else
    echo "LIne 'source /opt/ros/humble/setup.bash' already present in .bashrc"

fi