#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [[ "${USER:-}" != "root" ]]; then
  TARGET_USER="${USER}"
else
  TARGET_USER="${HOST_USERNAME:-lubobot}"
fi

ensure_group() {
  local gid="$1"
  local name="$2"

  if [[ -z "${gid}" || ! "${gid}" =~ ^[0-9]+$ ]]; then
    return 0
  fi

  if ! getent group "${gid}" >/dev/null; then
    groupadd -g "${gid}" "${name}" >/dev/null 2>&1 || true
  fi

  local group_name
  group_name="$(getent group "${gid}" | cut -d: -f1 || true)"
  if [[ -n "${group_name}" ]]; then
    usermod -aG "${group_name}" "${TARGET_USER}" >/dev/null 2>&1 || true
  fi
}

ensure_group "${AMD_RENDER_GID:-}" "render-host"
ensure_group "${AMD_VIDEO_GID:-}" "video-host"

if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
  mkdir -p "${XDG_RUNTIME_DIR}"
  chmod 700 "${XDG_RUNTIME_DIR}" || true
  chown "${TARGET_USER}:${TARGET_USER}" "${XDG_RUNTIME_DIR}" || true
fi

if [[ -f /workspaces/ros2_ws/install/setup.bash ]]; then
  source /workspaces/ros2_ws/install/setup.bash
fi

if [[ "$(id -un)" == "${TARGET_USER}" ]]; then
  exec "$@"
fi

exec gosu "${TARGET_USER}" "$@"
