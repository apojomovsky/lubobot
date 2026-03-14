#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
EXTERNAL_DIR="${ROOT_DIR}/external"
REPO_DIR="${EXTERNAL_DIR}/ros2_orb_slam3"
DEFAULT_BRANCH="humble-lubobot"
UPSTREAM_URL="https://github.com/Mechazo11/ros2_orb_slam3.git"

mkdir -p "${EXTERNAL_DIR}" "${ROOT_DIR}/ros2_ws/build" "${ROOT_DIR}/ros2_ws/install" "${ROOT_DIR}/ros2_ws/log"

if [[ ! -d "${REPO_DIR}/.git" ]]; then
  if [[ $# -lt 1 ]]; then
    echo "ERROR: ${REPO_DIR} does not exist." >&2
    echo "Provide your fork URL to clone it:" >&2
    echo "  scripts/setup_external.sh <fork-url> [branch]" >&2
    exit 1
  fi

  FORK_URL="$1"
  BRANCH="${2:-$DEFAULT_BRANCH}"
  git clone "${FORK_URL}" "${REPO_DIR}"
  git -C "${REPO_DIR}" checkout "${BRANCH}"
else
  BRANCH="${1:-$DEFAULT_BRANCH}"
  current_branch="$(git -C "${REPO_DIR}" rev-parse --abbrev-ref HEAD)"
  if [[ "${current_branch}" != "${BRANCH}" ]]; then
    git -C "${REPO_DIR}" checkout "${BRANCH}"
  fi
fi

if ! git -C "${REPO_DIR}" remote get-url upstream >/dev/null 2>&1; then
  git -C "${REPO_DIR}" remote add upstream "${UPSTREAM_URL}"
fi

echo "✓ external dependency ready at ${REPO_DIR}"
echo "✓ workspace cache directories ready under ${ROOT_DIR}/ros2_ws"
