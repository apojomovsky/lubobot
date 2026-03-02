#!/usr/bin/env bash
# deploy.sh — rsync local src/ to the Jetson workspace
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# Load config
if [[ -f "$ROOT_DIR/.env" ]]; then
  source "$ROOT_DIR/.env"
else
  echo "ERROR: .env not found. Copy .env.example to .env and fill it in." >&2
  exit 1
fi

REMOTE="${JETSON_USER}@${JETSON_HOST}"
REMOTE_WS="~/${JETSON_WORKSPACE}"

echo "→ Deploying src/ to ${REMOTE}:${REMOTE_WS}/src/"

# Ensure remote workspace/src exists
ssh "$REMOTE" "mkdir -p ${REMOTE_WS}/src"

# Sync src/ (exclude any local build artifacts that leaked in)
rsync -avz --delete \
  --exclude='__pycache__' \
  --exclude='*.pyc' \
  --exclude='*.egg-info' \
  "$ROOT_DIR/src/" \
  "${REMOTE}:${REMOTE_WS}/src/"

echo "✓ Deploy done"
