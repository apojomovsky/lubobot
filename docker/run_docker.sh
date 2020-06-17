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
    -v ${REPO_DIR}/lubobot:/catkin_ws/src/lubobot"
DOCKER_GRAPHICS_FLAG="--device /dev/dri"

xhost +
docker run --privileged --rm \
        ${DOCKER_CAPABILITIES} \
        ${DOCKER_MOUNT_ARGS} \
        -v /etc/fstab:/etc/fstab:ro \
        -e ROS_HOSTNAME=localhost \
        -e ROS_MASTER_URI=http://localhost:11311 \
        ${DOCKER_GRAPHICS_FLAG} \
        --device /dev/ttyACM0 \
        ${DOCKER_NETWORK} \
        -e DISPLAY=${DISPLAY} \
        -it ros-kinetic-dev
xhost -
