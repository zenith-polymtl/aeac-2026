#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:?Usage: $0 <workspace_dir>}"
PKGLIST="$WS_DIR/pkgs.txt"
SRC="$WS_DIR/src"

[ -f "$PKGLIST" ] || { echo "Missing $PKGLIST"; exit 1; }
mkdir -p "$SRC"

# repo root (works whether inside meta-repo or via symlink)
ROOT="$(git rev-parse --show-toplevel 2>/dev/null || realpath "$(cd "$(dirname "$0")/.." && pwd)")"

# remove old symlinks only
find "$SRC" -mindepth 1 -maxdepth 1 -type l -exec rm -f {} \;

while IFS= read -r PKG; do
  [[ -z "$PKG" || "$PKG" =~ ^# ]] && continue
  PKG_PATH="$ROOT/packages/$PKG"
  [ -d "$PKG_PATH" ] || { echo "ERROR: $PKG_PATH not found"; exit 2; }

  # make a RELATIVE symlink so it works both on host and in container
  REL_TARGET="$(realpath --relative-to="$SRC" "$PKG_PATH" 2>/dev/null || python3 - <<PY
import os,sys
print(os.path.relpath("$PKG_PATH","$SRC"))
PY
)"
  ln -s "$REL_TARGET" "$SRC/$PKG"
  echo "linked $PKG"
done < "$PKGLIST"

echo "Workspace $WS_DIR is set up."
