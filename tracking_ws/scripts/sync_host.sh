#!/usr/bin/env bash
# Push this workspace's code to the lab host PC and build it there. Nothing runs on this laptop.
#   scripts/sync_host.sh            sync + build
#   scripts/sync_host.sh -n         dry run: list what would change, touch nothing
# Build outputs, weights, logs and the host's .venv are never copied or deleted.
set -e
HOST=${TRACK_HOST:-drone@160.30.63.101}
PORT=${TRACK_PORT:-2222}
DEST=${TRACK_DEST:-ALBATROSS/tracking_ws}
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DRY=""
[ "$1" = "-n" ] && DRY="--dry-run"

# one SSH connection (one password prompt) for the sync and the build
CTL="$(mktemp -u /tmp/sync_host.XXXXXX)"
SSH="ssh -p $PORT -o ControlMaster=auto -o ControlPath=$CTL -o ControlPersist=60"
trap '$SSH -O exit "$HOST" 2>/dev/null || true' EXIT

rsync -az --checksum --itemize-changes $DRY -e "$SSH" \
  --exclude build/ --exclude install/ --exclude log/ --exclude assets/ --exclude .venv/ \
  --exclude __pycache__/ --exclude '*.pyc' --exclude .pytest_cache/ --exclude .vscode/ --exclude '.~lock*' \
  "$WS/" "$HOST:$DEST/" | grep -v '^\.d' || true

[ -n "$DRY" ] && exit 0
$SSH "$HOST" "cd $DEST && scripts/build.sh 2>&1 | tail -3"
