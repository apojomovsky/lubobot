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

detect_input() {
  if [[ -n "${ROS_INPUT:-}" ]]; then
    case "${ROS_INPUT}" in
      1|true|yes|on)
        echo "input"
        return 0
        ;;
      0|false|no|off)
        echo "none"
        return 0
        ;;
    esac
  fi

  if compgen -G "/dev/input/js*" >/dev/null; then
    echo "input"
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
  export SERIAL_GIDS="${SERIAL_GIDS:-}"
  if [[ -z "${SERIAL_GIDS}" ]]; then
    local -a serial_nodes=()
    if [[ -d /dev/serial/by-id ]]; then
      while IFS= read -r serial_node; do
        serial_nodes+=("${serial_node}")
      done < <(find /dev/serial/by-id -maxdepth 1 -type l | sort)
    fi
    if compgen -G "/dev/ttyACM*" >/dev/null; then
      while IFS= read -r serial_node; do
        serial_nodes+=("${serial_node}")
      done < <(printf '%s\n' /dev/ttyACM* | sort)
    fi
    if compgen -G "/dev/ttyUSB*" >/dev/null; then
      while IFS= read -r serial_node; do
        serial_nodes+=("${serial_node}")
      done < <(printf '%s\n' /dev/ttyUSB* | sort)
    fi

    local -a serial_gids=()
    local serial_gid=""
    for serial_node in "${serial_nodes[@]}"; do
      serial_gid="$(stat -Lc '%g' "${serial_node}" 2>/dev/null || true)"
      if [[ -n "${serial_gid}" ]] && [[ ! " ${serial_gids[*]} " =~ [[:space:]]${serial_gid}[[:space:]] ]]; then
        serial_gids+=("${serial_gid}")
      fi
    done

    if [[ "${#serial_gids[@]}" -gt 0 ]]; then
      export SERIAL_GIDS
      SERIAL_GIDS="$(IFS=,; printf '%s' "${serial_gids[*]}")"
      if [[ -z "${SERIAL_GID}" ]]; then
        export SERIAL_GID="${serial_gids[0]}"
      fi
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

  export INPUT_GID="${INPUT_GID:-}"
  if [[ -z "${INPUT_GID}" ]]; then
    local input_node=""
    if compgen -G "/dev/input/js*" >/dev/null; then
      input_node="$(printf '%s\n' /dev/input/js* | head -n 1)"
    fi
    if [[ -n "${input_node}" ]]; then
      export INPUT_GID="$(stat -Lc '%g' "${input_node}" 2>/dev/null || true)"
    fi
  fi
}

compose_files() {
  local gpu="$1"
  local root_dir="$2"
  local serial="${3:-none}"
  local video="${4:-none}"
  local input="${5:-none}"

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

  if [[ "${serial}" == "serial" || "${video}" == "video" || "${input}" == "input" ]]; then
    printf -- " -f %s/docker/compose.humble.devices.yml" "${root_dir}"
  fi
}

container_exists() {
  docker container inspect lubobot-humble-dev >/dev/null 2>&1
}

container_running() {
  local running="false"
  running="$(docker inspect -f '{{.State.Running}}' lubobot-humble-dev 2>/dev/null || echo false)"
  [[ "${running}" == "true" ]]
}
