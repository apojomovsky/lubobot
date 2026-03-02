#!/usr/bin/env bash
# ssh.sh — open a shell on the Jetson
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

if [[ -f "$ROOT_DIR/.env" ]]; then
  source "$ROOT_DIR/.env"
else
  echo "ERROR: .env not found." >&2
  exit 1
fi

exec ssh "${JETSON_USER}@${JETSON_HOST}" "$@"
