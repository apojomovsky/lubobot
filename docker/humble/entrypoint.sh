#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
  mkdir -p "${XDG_RUNTIME_DIR}"
  chmod 700 "${XDG_RUNTIME_DIR}" || true
fi

if [[ -f /workspaces/lubobot/install/setup.bash ]]; then
  source /workspaces/lubobot/install/setup.bash
fi

exec "$@"
