#!/bin/bash

# check whether the autopilot source code is included in the drone/px4 folder
if [ ! -d ./dronesim/drone/px4 ]; then

    if [ ! -d ./dronesim/drone/px4 ]; then
        mkdir -p ./dronesim/drone/px4
    fi

    cd dronesim/drone
    git clone https://github.com/PX4/PX4-Autopilot.git px4
    cd px4
    git checkout tags/v1.14.0-beta2
    cd ../../..

fi

# check whether the px4_msgs source code is included in the drone/onboard folder
if [ ! -d ./dronesim/drone/onboard/src/px4_msgs ]; then

    if [ ! -d ./dronesim/drone/onboard/src/px4_msgs ]; then
        mkdir -p ./dronesim/drone/onboard/src/px4_msgs
    fi

    cd dronesim/drone/onboard/src
    git clone https://github.com/PX4/px4_msgs.git px4_msgs
    cd ../../../..

fi

# enable the communication between containers and X windows in the host
xhost +local:docker

# set the current user name/group/uid/gid as environment variables
export CONTAINER_USER_NAME=$(id -un)
export CONTAINER_USER_ID=$(id -u)
export CONTAINER_GROUP_NAME=$(id -gn)
export CONTAINER_GROUP_ID=$(id -g)