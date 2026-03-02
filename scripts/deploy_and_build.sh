#!/usr/bin/env bash
# deploy_and_build.sh — deploy then build in one shot
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/deploy.sh"
"$SCRIPT_DIR/build.sh"
