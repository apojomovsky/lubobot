#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
OUT_DIR="${ROOT_DIR}/docker/rosdep-manifests"

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"
touch "${OUT_DIR}/COLCON_IGNORE"

copy_manifest() {
  local src_file="$1"
  local rel_path="$2"
  local dst_dir="${OUT_DIR}/${rel_path}"
  mkdir -p "${dst_dir}"
  cp "${src_file}" "${dst_dir}/package.xml"
}

if [[ -d "${ROOT_DIR}/src" ]]; then
  while IFS= read -r package_xml; do
    rel="${package_xml#${ROOT_DIR}/src/}"
    rel="$(dirname "${rel}")"
    copy_manifest "${package_xml}" "src/${rel}"
  done < <(find "${ROOT_DIR}/src" -name package.xml | sort)
fi

if [[ -d "${ROOT_DIR}/external/ros2_orb_slam3" ]]; then
  while IFS= read -r package_xml; do
    rel="${package_xml#${ROOT_DIR}/external/ros2_orb_slam3/}"
    rel="$(dirname "${rel}")"
    copy_manifest "${package_xml}" "external/ros2_orb_slam3/${rel}"
  done < <(find "${ROOT_DIR}/external/ros2_orb_slam3" -name package.xml ! -path '*/Thirdparty/*' | sort)
fi

echo "✓ synced rosdep manifests to ${OUT_DIR}"
