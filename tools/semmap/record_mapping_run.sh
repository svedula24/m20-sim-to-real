#!/usr/bin/env bash
# Offline phase, step 1: drive a coverage tour of the site and record keyframes for the object map.
# Run with the robot stack up (tools/robot/robot_up.sh with a site map and AMCL; robot armed ASSISTED) and the
# RGB-D camera publishing /camera/image_raw (rgb8), /camera/depth (32FC1 metres) and /camera/camera_info with
# identical stamps on colour and depth, plus the TF base_link -> camera_optical_link (tools/robot/robot_up.sh).
#
#   bash tools/semmap/record_mapping_run.sh --map maps/<site>.yaml [--site <site>] [--spacing 3.0] [--resume]
#   -> ~/m20_sim/semmap/<site>/run_<timestamp>/ (keyframes), then build the map with
#      bash tools/semmap/build_object_map.sh ~/m20_sim/semmap/<site>/run_<timestamp> [more runs...]
# --resume: drive only the tour stops that no earlier run of this site passed within 2 m of (nearest first
#   from the robot), e.g. after a run that ended early; then build from all the runs together.
# The start pose is the robot's localized pose (TF map -> base_link), read inside the commander container.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
C=${M20_CONTAINER:-docker-commander-1}
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
MAP_YAML=""; SITE=""; SPACING=3.0; RESUME=0
while [ $# -gt 0 ]; do
  case "$1" in
    --map) MAP_YAML="$2"; shift ;;
    --site) SITE="$2"; shift ;;
    --spacing) SPACING="$2"; shift ;;
    --resume) RESUME=1 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
[ -n "$MAP_YAML" ] && [ -f "$MAP_YAML" ] || { echo "usage: record_mapping_run.sh --map maps/<site>.yaml (a Nav2 map_server pgm/yaml pair)"; exit 2; }
MAP_YAML="$(cd "$(dirname "$MAP_YAML")" && pwd)/$(basename "$MAP_YAML")"
[ -n "$SITE" ] || SITE="$(basename "${MAP_YAML%.yaml}")"
SITE_DIR="$HOME/m20_sim/semmap/$SITE"
SKIP=()
if [ "$RESUME" = "1" ]; then
  for d in "$SITE_DIR"/run_*/; do                        # earlier runs with real data (not aborted stubs)
    [ "$(wc -l < "$d/index.jsonl" 2>/dev/null || echo 0)" -gt 20 ] && SKIP+=("/site/$(basename "$d")")
  done
  [ ${#SKIP[@]} -gt 0 ] || { echo "ERROR: --resume but no earlier run with keyframes in $SITE_DIR"; exit 1; }
fi
RUN="$SITE_DIR/run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN"

# Start pose from the localized TF, not from a simulator API: fails loudly when localization is not up.
pose=$(docker exec -i "$C" bash -lc "$SRC; timeout 15 python3 -" <<'PY'
import sys, rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("semmap_start_pose"); buf = Buffer(); TransformListener(buf, n)
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.1)
    if buf.can_transform("map", "base_link", Time()):
        t = buf.lookup_transform("map", "base_link", Time()).transform.translation
        print(f"{t.x:.2f} {t.y:.2f}"); sys.exit(0)
print("ERROR: no TF map -> base_link after 10 s (is localization up and initialized?)", file=sys.stderr); sys.exit(1)
PY
)
echo "[mapping] robot at $pose (map frame); planning a coverage tour (spacing $SPACING m)"
# shellcheck disable=SC2086  # $pose is two numbers on purpose
mapfile -t STOPS < <(docker run --rm -v "$REPO":/repo:ro -v "$(dirname "$MAP_YAML")":/map:ro -v "$SITE_DIR":/site:ro m20_nlnav:latest \
  python3 /repo/tools/semmap/coverage_tour.py "/map/$(basename "$MAP_YAML")" --start $pose --spacing "$SPACING" \
  ${SKIP[@]:+--skip-seen "${SKIP[@]}"})
echo "[mapping] ${#STOPS[@]} stops"
[ ${#STOPS[@]} -gt 0 ] || { echo "[mapping] nothing left to cover"; rmdir "$RUN"; exit 0; }

docker rm -f m20_semmap_rec >/dev/null 2>&1 || true
docker run -d --name m20_semmap_rec --network host --ipc host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e PYTHONUNBUFFERED=1 -e SEMMAP_OUT=/out \
  -v "$REPO/tools/semmap":/semmap:ro -v "$RUN":/out \
  m20_nlnav:latest bash -lc "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /semmap/keyframe_recorder.py" >/dev/null
trap 'docker rm -f m20_semmap_rec >/dev/null 2>&1 || true' EXIT
sleep 4
docker ps --format '{{.Names}}' | grep -q '^m20_semmap_rec$' || { echo "ERROR: keyframe recorder exited:"; docker logs m20_semmap_rec | tail -20; exit 1; }

echo "[mapping] driving the tour (one mission; Ctrl-C to abort)"
"$REPO/tools/dev/m20" mission "${STOPS[@]}" || echo "[mapping] mission ended early: keeping what was recorded"
sleep 2
N=$(wc -l < "$RUN/index.jsonl" 2>/dev/null || echo 0)
echo "[mapping] $N keyframes in $RUN"
echo "next: bash tools/semmap/build_object_map.sh $RUN"
