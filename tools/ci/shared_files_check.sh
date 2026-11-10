#!/usr/bin/env bash
# Every file in tools/ci/shared_files.txt must be byte-identical to the ROS2_Nav2_Mako original.
#   bash tools/ci/shared_files_check.sh                 clones Mako (MAKO_REPO, MAKO_REF) at depth 1 and compares
#   MAKO_DIR=../ROS2_Nav2_Mako bash tools/ci/shared_files_check.sh    compares against a local checkout instead
# Exit 1 with the list of differing or missing files; nothing is changed on either side.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
MAKO_REPO="${MAKO_REPO:-https://github.com/svedula24/ROS2_Nav2_Mako.git}"
MAKO_REF="${MAKO_REF:-3dgs-sim-nl-navigation-offline}"
if [ -z "${MAKO_DIR:-}" ]; then
  MAKO_DIR="$(mktemp -d)"
  trap 'rm -rf "$MAKO_DIR"' EXIT
  git clone -q --depth 1 --branch "$MAKO_REF" "$MAKO_REPO" "$MAKO_DIR"
fi
bad=0; n=0
while read -r f; do
  case "$f" in ''|'#'*) continue ;; esac
  n=$((n + 1))
  if [ ! -f "$MAKO_DIR/$f" ]; then echo "MISSING in Mako: $f"; bad=1
  elif [ ! -f "$REPO/$f" ]; then echo "MISSING here:    $f"; bad=1
  elif ! cmp -s "$MAKO_DIR/$f" "$REPO/$f"; then echo "DIFFERS:         $f"; bad=1
  fi
done < "$REPO/tools/ci/shared_files.txt"
[ "$n" -gt 0 ] || { echo "shared_files.txt lists no files"; exit 1; }
[ "$bad" = 0 ] && echo "shared files: $n identical to Mako ($MAKO_REF)"
exit "$bad"
