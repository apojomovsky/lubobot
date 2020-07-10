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
# REMOTE_ARGS=""
REMOTE_ARGS="-e ROS_MASTER_URI=http://`hostname -I | cut -d " " -f1`:11311 \
  -e ROS_HOSTNAME=`hostname -I | cut -d " " -f1` \
  -e ROS_IP=`hostname -I | cut -d " " -f1`"


xhost +
docker run ${DOCKER_ULIMIT_ARGS} --privileged --rm \
        ${DOCKER_CAPABILITIES} \
        ${DOCKER_MOUNT_ARGS} \
        -v /etc/fstab:/etc/fstab:ro \
        ${DOCKER_GRAPHICS_FLAG} \
        ${REMOTE_ARGS} \
        --device /dev/ttyACM0 \
        --device /dev/ttyUSB0 \
        ${DOCKER_NETWORK} \
        -e DISPLAY=${DISPLAY} \
        -it ros-kinetic-dev
xhost -
