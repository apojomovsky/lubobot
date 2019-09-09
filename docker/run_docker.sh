SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
REPO_DIR=`readlink -f ${SCRIPTS_DIR}/..`

DOCKER_CAPABILITIES="--ipc=host \
                     --cap-add=IPC_LOCK \
                     --cap-add=sys_nice"
DOCKER_NETWORK="--network=host"
DOCKER_MOUNT_ARGS="\
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ${REPO_DIR}:/catkin_ws/src/tesis-bot"
DOCKER_GRAPHICS_FLAG="--device /dev/dri"

xhost +
docker run --privileged --rm \
        ${DOCKER_CAPABILITIES} \
        ${DOCKER_MOUNT_ARGS} \
        -v /etc/fstab:/etc/fstab:ro \
        -e ROS_HOSTNAME=localhost \
        -e ROS_MASTER_URI=http://localhost:11311 \
        ${DOCKER_GRAPHICS_FLAG} \
        ${DOCKER_NETWORK} \
        -e DISPLAY=${DISPLAY} \
        -it ros-kinetic-dev
xhost -
