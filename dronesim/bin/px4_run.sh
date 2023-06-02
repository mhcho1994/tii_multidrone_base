#!/bin/bash

# show commands before execution and exit when errors occur
set -e -x

# check whether px4 is already built
if [ ! -d ~/px4/build ]; then

    cd ~/px4
    make px4_sitl
    cd ..
fi

# run px4 and gazebo simulation
cd ~/px4
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE_1} PX4_GZ_MODEL=${PX4_GZ_MODEL_1} ./build/px4_sitl_default/bin/px4 -i ${INSTANCE_SIGN_1}