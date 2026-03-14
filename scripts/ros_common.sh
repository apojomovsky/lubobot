#!/usr/bin/env bash

detect_gpu() {
  if [[ -n "${HUMBLE_GPU:-}" ]]; then
    echo "${HUMBLE_GPU}"
    return 0
  fi

  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    echo "nvidia"
    return 0
  fi

  if [[ -e /dev/kfd && -d /dev/dri ]]; then
    echo "amd"
    return 0
  fi

  echo "cpu"
}

detect_serial() {
  if [[ -n "${ROS_SERIAL:-}" ]]; then
    case "${ROS_SERIAL}" in
      1|true|yes|on)
        echo "serial"
        return 0
        ;;
      0|false|no|off)
        echo "none"
        return 0
        ;;
    esac
  fi

  if [[ -d /dev/serial/by-id ]] || compgen -G "/dev/ttyACM*" >/dev/null || compgen -G "/dev/ttyUSB*" >/dev/null; then
    echo "serial"
    return 0
  fi

  echo "none"
}

detect_video() {
  if [[ -n "${ROS_VIDEO:-}" ]]; then
    case "${ROS_VIDEO}" in
      1|true|yes|on)
        echo "video"
        return 0
        ;;
      0|false|no|off)
        echo "none"
        return 0
        ;;
    esac
  fi

  if [[ -d /dev/v4l/by-id ]] || compgen -G "/dev/video*" >/dev/null; then
    echo "video"
    return 0
  fi

  echo "none"
}

setup_gpu_env() {
  local gpu="$1"

  export HOST_UID="${HOST_UID:-$(id -u)}"
  export HOST_GID="${HOST_GID:-$(id -g)}"
  export HOST_USERNAME="${HOST_USERNAME:-$(id -un)}"
  export DISPLAY="${DISPLAY:-:0}"
  export QT_X11_NO_MITSHM=1
  export XDG_RUNTIME_DIR="/tmp/runtime-root"
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

  case "${gpu}" in
    amd)
      export AMD_RENDER_GID="$(stat -c '%g' /dev/dri/renderD128 2>/dev/null || printf 'render')"
      export AMD_VIDEO_GID="$(stat -c '%g' /dev/dri/card0 2>/dev/null || printf 'video')"
      export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/radeon_icd.json}"
      ;;
    nvidia|cpu)
      :
      ;;
    *)
      echo "ERROR: unsupported GPU mode '${gpu}'" >&2
      return 1
      ;;
  esac

  export SERIAL_GID="${SERIAL_GID:-}"
  if [[ -z "${SERIAL_GID}" ]]; then
    local serial_node=""
    if [[ -d /dev/serial/by-id ]]; then
      serial_node="$(find /dev/serial/by-id -maxdepth 1 -type l | head -n 1)"
    fi
    if [[ -z "${serial_node}" ]] && compgen -G "/dev/ttyACM*" >/dev/null; then
      serial_node="$(printf '%s\n' /dev/ttyACM* | head -n 1)"
    fi
    if [[ -z "${serial_node}" ]] && compgen -G "/dev/ttyUSB*" >/dev/null; then
      serial_node="$(printf '%s\n' /dev/ttyUSB* | head -n 1)"
    fi
    if [[ -n "${serial_node}" ]]; then
      export SERIAL_GID="$(stat -Lc '%g' "${serial_node}" 2>/dev/null || true)"
    fi
  fi

  export VIDEO_GID="${VIDEO_GID:-}"
  if [[ -z "${VIDEO_GID}" ]]; then
    local video_node=""
    if [[ -d /dev/v4l/by-id ]]; then
      video_node="$(find /dev/v4l/by-id -maxdepth 1 -type l | head -n 1)"
    fi
    if [[ -z "${video_node}" ]] && compgen -G "/dev/video*" >/dev/null; then
      video_node="$(printf '%s\n' /dev/video* | head -n 1)"
    fi
    if [[ -n "${video_node}" ]]; then
      export VIDEO_GID="$(stat -Lc '%g' "${video_node}" 2>/dev/null || true)"
    fi
  fi
}

compose_files() {
  local gpu="$1"
  local root_dir="$2"
  local serial="${3:-none}"
  local video="${4:-none}"

  printf -- "-f %s/docker/compose.humble.yml " "${root_dir}"

  case "${gpu}" in
    amd)
      printf -- "-f %s/docker/compose.humble.amd.yml" "${root_dir}"
      ;;
    nvidia)
      printf -- "-f %s/docker/compose.humble.nvidia.yml" "${root_dir}"
      ;;
    cpu)
      ;;
  esac

  if [[ "${serial}" == "serial" || "${video}" == "video" ]]; then
    printf -- " -f %s/docker/compose.humble.devices.yml" "${root_dir}"
  fi
}
