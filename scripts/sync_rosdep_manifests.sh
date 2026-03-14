#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
OUT_DIR="${ROOT_DIR}/docker/rosdep-manifests"
APT_PKG_FILE="${ROOT_DIR}/docker/rosdep-apt-packages.txt"
STATE_HASH_FILE="${ROOT_DIR}/docker/rosdep-inputs.sha256"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-libcrypto boost_serialization}"

copy_manifest() {
  local src_file="$1"
  local rel_path="$2"
  local dst_dir="${OUT_DIR}/${rel_path}"
  mkdir -p "${dst_dir}"
  cp "${src_file}" "${dst_dir}/package.xml"
}

declare -a package_xmls=()

if [[ -d "${ROOT_DIR}/src" ]]; then
  while IFS= read -r package_xml; do
    package_xmls+=("${package_xml}")
    rel="${package_xml#${ROOT_DIR}/src/}"
    rel="$(dirname "${rel}")"
    copy_manifest "${package_xml}" "src/${rel}"
  done < <(find "${ROOT_DIR}/src" -name package.xml | sort)
fi

if [[ -d "${ROOT_DIR}/external/ros2_orb_slam3" ]]; then
  while IFS= read -r package_xml; do
    package_xmls+=("${package_xml}")
    rel="${package_xml#${ROOT_DIR}/external/ros2_orb_slam3/}"
    rel="$(dirname "${rel}")"
    copy_manifest "${package_xml}" "external/ros2_orb_slam3/${rel}"
  done < <(find "${ROOT_DIR}/external/ros2_orb_slam3" -name package.xml ! -path '*/Thirdparty/*' | sort)
fi

compute_inputs_hash() {
  if [[ "${#package_xmls[@]}" -eq 0 ]]; then
    printf 'empty\n'
    return 0
  fi

  {
    printf 'skip-keys:%s\n' "${ROSDEP_SKIP_KEYS}"
    for package_xml in "${package_xmls[@]}"; do
      sha256sum "${package_xml}"
    done
  } | sha256sum | awk '{print $1}'
}

INPUTS_HASH="$(compute_inputs_hash)"

if [[ -f "${STATE_HASH_FILE}" ]] && [[ -f "${APT_PKG_FILE}" ]] && [[ -d "${OUT_DIR}" ]] && [[ "$(cat "${STATE_HASH_FILE}")" == "${INPUTS_HASH}" ]]; then
  echo "✓ rosdep manifest cache is up to date"
  exit 0
fi

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"
touch "${OUT_DIR}/COLCON_IGNORE"
rm -f "${APT_PKG_FILE}"

declare -A workspace_packages=()
declare -A skip_keys=()
declare -A dep_keys=()
declare -A apt_packages=()

for key in ${ROSDEP_SKIP_KEYS}; do
  skip_keys["${key}"]=1
done

for package_xml in "${package_xmls[@]}"; do
  pkg_name="$(sed -n 's:.*<name>\([^<]\+\)</name>.*:\1:p' "${package_xml}" | head -n 1)"
  if [[ -n "${pkg_name}" ]]; then
    workspace_packages["${pkg_name}"]=1
  fi
done

resolve_apt_packages() {
  local dep_key="$1"
  ROS_DISTRO= rosdep resolve "${dep_key}" --rosdistro humble --os ubuntu:jammy 2>/dev/null | awk '
    /^#apt$/ { in_apt=1; next }
    /^#/ { in_apt=0 }
    in_apt && NF { print }
  '
}

for package_xml in "${package_xmls[@]}"; do
  while IFS= read -r dep_key; do
    [[ -z "${dep_key}" ]] && continue
    if [[ -n "${workspace_packages[${dep_key}]:-}" || -n "${skip_keys[${dep_key}]:-}" ]]; then
      continue
    fi
    dep_keys["${dep_key}"]=1
  done < <(rg -oP '<(?:depend|build_depend|build_export_depend|buildtool_depend|exec_depend)>\K[^<]+' "${package_xml}" | sort -u)
done

for dep_key in "${!dep_keys[@]}"; do
  while IFS= read -r apt_pkg; do
    [[ -z "${apt_pkg}" ]] && continue
    apt_packages["${apt_pkg}"]=1
  done < <(resolve_apt_packages "${dep_key}")
done

for apt_pkg in "${!apt_packages[@]}"; do
  printf '%s\n' "${apt_pkg}"
done | sort > "${APT_PKG_FILE}"

printf '%s\n' "${INPUTS_HASH}" > "${STATE_HASH_FILE}"

echo "✓ synced rosdep manifests to ${OUT_DIR}"
echo "✓ wrote ROS apt package list to ${APT_PKG_FILE}"
