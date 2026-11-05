#!/usr/bin/env bash
# (Re)start the natural-language navigation container (m20_nlnav) next to the robot stack.
#   bash tools/nlnav/nlnav_up.sh --object-map ~/m20_sim/semmap/<site>/object_map.json --scene maps/<site>.scene.yaml
#        [--llm ollama|claude|json] [--model qwen2.5:7b-instruct] [--min-label-score 0.4] [--wait] [--no-nav2]
# --no-nav2 (stage A, vendor navigation): the approach pose is checked on the static /map instead of Nav2's global
#   costmap, and the reachability check (Nav2's planner) is off.
# The pose source is always TF (map -> base_link from AMCL): there is no ground-truth pose on a robot.
# --wait blocks until the node logs "warm-up done" (text encoder + one real LLM parse), up to 3 min, and fails
# otherwise, so a caller never measures or trusts an unready node.
# The local LLM is Ollama in the m20_ollama container (tools/nlnav/ollama_up.sh); NLNAV_LLM=claude uses the API.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
LLM="${NLNAV_LLM:-ollama}"; MODEL="${NLNAV_LLM_MODEL:-qwen2.5:7b-instruct}"; MIN_LABEL="${MIN_LABEL_SCORE:-0.4}"
OBJECT_MAP=""; SCENE=""; WAIT=0; NO_NAV2=0
while [ $# -gt 0 ]; do
  case "$1" in
    --object-map) OBJECT_MAP="$2"; shift ;;
    --scene) SCENE="$2"; shift ;;
    --llm) LLM="$2"; shift ;;
    --model) MODEL="$2"; shift ;;
    --min-label-score) MIN_LABEL="$2"; shift ;;
    --wait) WAIT=1 ;;
    --no-nav2) NO_NAV2=1 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
[ -n "$OBJECT_MAP" ] && [ -n "$SCENE" ] || { echo "usage: nlnav_up.sh --object-map FILE --scene FILE.scene.yaml [...]"; exit 2; }
[ -f "$OBJECT_MAP" ] || { echo "ERROR: object map $OBJECT_MAP not found (tools/semmap/build_object_map.sh)"; exit 1; }
[ -f "$SCENE" ] || { echo "ERROR: scene yaml $SCENE not found (named waypoints of the site)"; exit 1; }
OBJECT_MAP="$(cd "$(dirname "$OBJECT_MAP")" && pwd)/$(basename "$OBJECT_MAP")"
SCENE="$(cd "$(dirname "$SCENE")" && pwd)/$(basename "$SCENE")"
NL_ENV=(-e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e PYTHONUNBUFFERED=1
        -e HF_HUB_OFFLINE=1 -e TRANSFORMERS_OFFLINE=1 -e NLNAV_LLM="$LLM" -e NLNAV_LLM_MODEL="$MODEL")
[ -n "${ANTHROPIC_API_KEY:-}" ] && NL_ENV+=(-e ANTHROPIC_API_KEY)
NL_CMD="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export PYTHONPATH=/nlnav:\$PYTHONPATH;"
NL_CMD+=" python3 -c 'from m20_nlnav.nlnav_node import main; main()' --ros-args"
NL_CMD+=" -p scene_yaml:=/scene/$(basename "$SCENE") -p object_map:=/semmap/$(basename "$OBJECT_MAP")"
NL_CMD+=" -p llm:=$LLM -p text_encoder:=owl -p min_label_score:=$MIN_LABEL -p pose_source:=tf"
[ "$NO_NAV2" = 1 ] && NL_CMD+=" -p costmap_topic:=/map -p costmap_service:=/map_server/unavailable -p reach_check:=false"
docker rm -f m20_nlnav >/dev/null 2>&1 || true
docker run -d --name m20_nlnav --network host --ipc host --gpus all "${NL_ENV[@]}" \
  -v "$REPO/src/m20_nlnav":/nlnav:ro -v "$(dirname "$SCENE")":/scene:ro -v "$(dirname "$OBJECT_MAP")":/semmap:ro \
  m20_nlnav:latest bash -lc "$NL_CMD" >/dev/null
echo "[nlnav_up] m20_nlnav started: llm=$LLM model=$MODEL min_label_score=$MIN_LABEL pose_source=tf"
if [ "$WAIT" = "1" ]; then
  for _ in $(seq 1 90); do docker logs m20_nlnav 2>&1 | grep -qa "warm-up done\|warm-up failed\|Traceback" && break; sleep 2; done
  docker logs m20_nlnav 2>&1 | grep -aE "objects kept|warm-up|Traceback" | sed -E 's/^\[INFO\] \[[0-9.]+\] \[m20_nlnav\]: /[nlnav_up] /' | tail -3 || true
  docker logs m20_nlnav 2>&1 | grep -qa "warm-up done" ||
    { echo "[nlnav_up] ERROR: m20_nlnav not ready (warm-up failed, crashed, or > 180 s): docker logs m20_nlnav"; exit 1; }
fi
