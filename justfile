set shell := ["bash", "-euo", "pipefail", "-c"]

default:
  @just --list

setup-external fork-url='' branch='humble-lubobot':
  if [[ -n "{{fork-url}}" ]]; then \
    ./scripts/setup_external.sh "{{fork-url}}" "{{branch}}"; \
  else \
    ./scripts/setup_external.sh "{{branch}}"; \
  fi

sync-manifests:
  ./scripts/sync_rosdep_manifests.sh

ros-up:
  ./scripts/ros_up.sh

ros-shell:
  ./scripts/ros_shell.sh

ros-attach:
  ./scripts/ros_attach.sh

ros-build:
  ./scripts/ros_build.sh

ros-down:
  ./scripts/ros_down.sh

kilted-setup:
  ./scripts/setup_kilted_host_camera_lab.sh

kilted-build:
  ./scripts/build_kilted_host_camera_lab.sh

kilted-c920-calibrate:
  ./scripts/run_c920_calibration_host.sh
