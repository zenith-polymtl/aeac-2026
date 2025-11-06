#!/usr/bin/env bash
# Usage: scripts/link_ws.sh workspaces/ws_heavy_task_2
set -e
WS_DIR="$1"
[ -z "$WS_DIR" ] && { echo "Usage: $0 <workspace_dir>"; exit 1; }
PKGLIST="$WS_DIR/pkgs.txt"
SRC="$WS_DIR/src"

[ -f "$PKGLIST" ] || { echo "Missing $PKGLIST"; exit 1; }
mkdir -p "$SRC"

# Clean old links only (do not delete real dirs)
find "$SRC" -maxdepth 1 -mindepth 1 -type l -exec rm -f {} \;

# Create fresh links
while read -r PKG; do
  [ -z "$PKG" ] && continue
  if [ ! -d "packages/$PKG" ]; then
    echo "ERROR: packages/$PKG does not exist"
    exit 2
  fi
  ln -s "../../packages/$PKG" "$SRC/$PKG"
  echo "linked $PKG"
done < "$PKGLIST"

echo "Workspace $WS_DIR is set up."