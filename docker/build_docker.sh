#!/usr/bin/env bash

SCRIPTS_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
USERID=`id -u`

docker build --build-arg uid=$USERID -t ros-noetic-dev $SCRIPTS_DIR
