#!/usr/bin/env bash

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
REPO_DIR=`readlink -f ${SCRIPTS_DIR}/..`

DOCKER_CAPABILITIES="--ipc=host \
                     --cap-add=IPC_LOCK \
                     --cap-add=sys_nice"
DOCKER_NETWORK="--network=host"
DOCKER_MOUNT_ARGS="\
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ${REPO_DIR}/external:/catkin_ws/src/external \
    -v ${REPO_DIR}/lubobot:/catkin_ws/src/lubobot \
    -v ${REPO_DIR}/lubobot_msgs:/catkin_ws/src/lubobot_msgs"
DOCKER_GRAPHICS_FLAG="--device /dev/dri"
DOCKER_ULIMIT_ARGS="--ulimit core=-1"

xhost +
docker run ${DOCKER_ULIMIT_ARGS} --privileged --rm \
        ${DOCKER_CAPABILITIES} \
        ${DOCKER_MOUNT_ARGS} \
        -v /etc/fstab:/etc/fstab:ro \
        -e ROS_MASTER_URI=http://192.168.1.200:11311 \
        -e ROS_HOSTNAME=192.168.1.200 \
        -e ROS_IP=192.168.1.200 \
        ${DOCKER_GRAPHICS_FLAG} \
        --device /dev/ttyACM0 \
        --device /dev/ttyUSB0 \
        ${DOCKER_NETWORK} \
        -e DISPLAY=${DISPLAY} \
        -it ros-kinetic-dev
xhost -
