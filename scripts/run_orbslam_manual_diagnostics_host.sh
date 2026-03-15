#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/env_kilted_host.sh"

exec "${SCRIPT_DIR}/run_orbslam_manual_diagnostics.sh" "$@"
