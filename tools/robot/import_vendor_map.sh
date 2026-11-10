#!/usr/bin/env bash
# Copy one of the robot's own maps off its navigation computer (NOS) into maps/<local>.{pgm,yaml} for Nav2.
#   bash tools/robot/import_vendor_map.sh <user@nos-host> <map name on NOS> <local name>
#   e.g.  bash tools/robot/import_vendor_map.sh robot@10.21.31.106 upstairs2026-20240813-024228 upstairs2026
# Read-only on NOS (two scp downloads, the password typed by hand). The yaml's `image:` line is rewritten to the local
# pgm; `resolution` and `origin` are NOT touched: the frame of this map is the frame the robot reports its position
# in, and Nav2 must plan in that frame. The robot's maps are 0.1 m cells; anything else is refused.
set -euo pipefail
[ $# -eq 3 ] || { sed -n '2,5p' "$0"; exit 2; }
NOS="$1"; NAME="$2"; LOCAL="$3"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
DST="$REPO/maps"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
scp "$NOS:/var/opt/robot/data/maps/$NAME/occ_grid.pgm" "$NOS:/var/opt/robot/data/maps/$NAME/occ_grid.yaml" "$TMP/"
RES=$(sed -n 's/^resolution: *//p' "$TMP/occ_grid.yaml")
[ "$RES" = "0.1" ] || { echo "import_vendor_map: resolution is '$RES', expected 0.1 (the robot's maps); refusing"; exit 1; }
read -r W H < <(python3 - "$TMP/occ_grid.pgm" <<'PY'
import sys
with open(sys.argv[1], "rb") as f:
    assert f.readline().strip() == b"P5", "not a binary PGM"
    line = f.readline()
    while line.startswith(b"#"):
        line = f.readline()
    print(*line.split()[:2])
PY
)
cp "$TMP/occ_grid.pgm" "$DST/$LOCAL.pgm"
sed "s|^image:.*|image: $LOCAL.pgm|" "$TMP/occ_grid.yaml" > "$DST/$LOCAL.yaml"
echo "import_vendor_map: maps/$LOCAL.{pgm,yaml} from $NAME: ${W} x ${H} cells at ${RES} m = $(python3 -c "print(f'{$W*0.1:.1f} x {$H*0.1:.1f} m')"), origin $(sed -n 's/^origin: *//p' "$DST/$LOCAL.yaml")"
