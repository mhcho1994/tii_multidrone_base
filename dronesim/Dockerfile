FROM ubuntu:22.04
LABEL maintainer="Minhyun Cho <cho515@purdue.edu>"

# set default shell during Docker image build to bash
SHELL ["/bin/bash", "-l", "-c"]

# environment variables to set pacakge manager
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# environment variable to designate the directory where temporary files and sockets are stored and accessible
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker

# environment variables to enable the use of NVIDIA GPU
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# set the partition name for gazebo simulation
ENV GZ_PARTITION=tii_multidrone

# set default shell during Docker image build to bash
SHELL ["/bin/bash", "-l", "-c"]

# install base packages to run bash scripts
RUN apt-get -y update && \
    apt-get -y upgrade && \
    apt-get -y --quiet --no-install-recommends install \
    sudo \
    locales

# clear docker by removing unnecessary packages and emptying temporary folder
COPY /dronesim/install/clean.sh /tmp/clean.sh
RUN chmod +x /tmp/clean.sh
RUN bash /tmp/clean.sh

# set and generate system locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8
RUN locale-gen en_US.UTF-8 && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	export LANG=en_US.UTF-8

# get the host user, group information to setup user
ARG HOST_USER_NAME HOST_USER_ID HOST_GROUP_NAME HOST_GROUP_ID

# add groups before we do anything that might add a new group
ARG GID_INPUT=107
ARG GID_RENDER=110
RUN sudo groupadd -r -g $GID_INPUT input \
 && sudo groupadd -r -g $GID_RENDER render

# setup user
RUN groupadd --gid ${HOST_GROUP_ID} docker \
 && adduser --disabled-password --gecos '' docker --uid ${HOST_USER_ID} --gid ${HOST_GROUP_ID}\
 && usermod -a -G sudo,plugdev,dialout,input,render,video docker \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker

# install required dependencies and packages using scripts (docker build cache)
# base
COPY --chown=docker:docker /dronesim/install/base.sh /tmp/install/base.sh
RUN chmod +x /tmp/install/base.sh
RUN bash /tmp/install/base.sh

# ROS2 (Humble)
COPY --chown=docker:docker /dronesim/install/ros.sh /tmp/install/ros.sh
RUN chmod +x /tmp/install/ros.sh
RUN bash /tmp/install/ros.sh
RUN mkdir -p ~/onboard

# Gazebo garden (from binary)
COPY --chown=docker:docker /dronesim/install/gazebo.sh /tmp/install/gazebo.sh
RUN chmod +x /tmp/install/gazebo.sh
RUN bash /tmp/install/gazebo.sh

# PX4 (PX4-Autopilot+dependencies)
RUN mkdir -p /tmp/install/px4_setup
COPY --chown=docker:docker /dronesim/drone/px4/Tools/setup/ubuntu.sh /tmp/install/px4_setup/ubuntu.sh
COPY --chown=docker:docker /dronesim/drone/px4/Tools/setup/requirements.txt /tmp/install/px4_setup/requirements.txt
COPY --chown=docker:docker /dronesim/install/px4.sh /tmp/install/px4.sh
RUN bash /tmp/install/px4.sh
RUN mkdir -p ~/px4

# extra packages
COPY --chown=docker:docker /dronesim/install/extra.sh /tmp/install/extra.sh
RUN chmod +x /tmp/install/extra.sh
RUN bash /tmp/install/extra.sh

# clear docker by removing unnecessary packages and emptying temporary folder
RUN bash /tmp/clean.sh

# enable apt auto-completion by deleting autoclean task (speed up Dockerfile)
RUN sudo rm /etc/apt/apt.conf.d/docker-clean

# create XDG runtime dir
RUN mkdir /tmp/runtime-docker && sudo chmod 700 /tmp/runtime-docker

# run setup script
COPY --chown=docker:docker /dronesim/install/setup.sh /tmp/install/setup.sh
RUN chmod +x /tmp/install/setup.sh
RUN bash /tmp/install/setup.sh