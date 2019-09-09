SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
USERID=`id -u`

docker build --build-arg uid=$USERID -t ros-kinetic-dev $SCRIPTS_DIR
