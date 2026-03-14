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
  ./scripts/humble_up.sh

ros-shell:
  ./scripts/humble_shell.sh

ros-attach:
  ./scripts/humble_attach.sh

ros-build:
  ./scripts/humble_build.sh

ros-down:
  ./scripts/humble_down.sh
