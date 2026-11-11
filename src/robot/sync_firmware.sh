#!/usr/bin/env bash
# Refresh the vendored AGSG firmware from GitHub.
#
# The firmware is copied (not submodule'd) so a run needs no network and the exact
# revision that produced a score stays in this repository's history.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${AGSG_REPO:-https://github.com/ImSimao/FE-AGSG-S2026.git}"
REF="${AGSG_REF:-main}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "[sync] $REPO @ $REF"
git clone --depth 1 --branch "$REF" "$REPO" "$TMP/agsg" >/dev/null 2>&1

rm -rf "$ROOT_DIR/agsg"
mkdir -p "$ROOT_DIR/agsg"
cp -r "$TMP/agsg/src/." "$ROOT_DIR/agsg/"
rm -rf "$ROOT_DIR/agsg/.micropico" "$ROOT_DIR/agsg/SonarSlave/.micropico"

( cd "$TMP/agsg" && git log -1 --format='%H %ci %s' ) > "$ROOT_DIR/agsg/.sync-revision"
echo "[sync] done: $(cat "$ROOT_DIR/agsg/.sync-revision")"
echo "[sync] remember to re-run the simulator: the firmware changed under it."
