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
}

compose_files() {
  local gpu="$1"
  local root_dir="$2"

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
}
