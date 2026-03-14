#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [[ -f /workspaces/lubobot/install/setup.bash ]]; then
  source /workspaces/lubobot/install/setup.bash
fi

exec "$@"
