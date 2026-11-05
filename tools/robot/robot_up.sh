#!/usr/bin/env bash
# The Mako stack on the real M20, from the laptop wired to the robot. STAYS IN THE FOREGROUND: Ctrl-C tears the
# navigation layer down (the compose services stay up; `make down` stops them).
#
#   bash tools/robot/robot_up.sh --map maps/<site>.yaml                 stage A: vendor navigation under the mission
#                                                                        server (no Nav2, no lidar driver on the laptop)
#   M20_NAV=nav2 bash tools/robot/robot_up.sh --map maps/<site>.yaml    stage B: Nav2 on the vendor map, the two
#                                                                        RoboSense clouds -> /scan_front + /scan_rear
#   --object-map ~/m20_sim/semmap/<site>/object_map.json --scene maps/<site>.scene.yaml
#                                                                        also start NL navigation (tools/nlnav)
#   --no-arm      bring everything up but leave the commander disarmed (bench checks)
#   then:  tools/dev/m20 goto X Y     tools/dev/m20 say "go to the ladder"     tools/dev/m20 status
#
# The map is the robot's own: copy /var/opt/robot/data/maps/<name>/occ_grid.{pgm,yaml} from the navigation host
# (NOS, 10.21.31.106) into maps/<site>.{pgm,yaml}; its frame is the vendor `map` the bridge publishes poses in.
# Preconditions (docs/REAL_ROBOT.md): laptop on the wire at 10.21.31.100/24, robot standing in the velocity
# controller (tools/robot/m20_probe.py stand; gait flat; control), pendant app NOT driving, localization ok.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
C=${M20_CONTAINER:-docker-commander-1}
DOM="${ROS_DOMAIN_ID:-42}"
SRC="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=$DOM RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
NAV="${M20_NAV:-vendor}"
MAP=""; OBJECT_MAP=""; SCENE=""; ARM=1
while [ $# -gt 0 ]; do
  case "$1" in
    --map) MAP="$2"; shift ;;
    --object-map) OBJECT_MAP="$2"; shift ;;
    --scene) SCENE="$2"; shift ;;
    --no-arm) ARM=0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
[ -n "$MAP" ] && [ -f "$MAP" ] && [ -f "${MAP%.yaml}.pgm" ] || { echo "usage: robot_up.sh --map maps/<site>.yaml (with its .pgm next to it)"; exit 2; }
case "$NAV" in vendor|nav2) ;; *) echo "M20_NAV must be vendor or nav2"; exit 2 ;; esac
MAP="$(cd "$(dirname "$MAP")" && pwd)/$(basename "$MAP")"

cleanup() {
  echo; echo "[robot_up] shutting the navigation layer down"
  bash "$REPO/tools/nav/kill_stack.sh" || true
}
trap cleanup EXIT INT TERM

echo "[1/6] compose services (commander, locomotion bridge -> robot, mission server, station)"
M20_VENDOR_NAV=$([ "$NAV" = vendor ] && echo true || echo false) M20_PUBLISH_MAP_TF=true \
  docker compose -f "$REPO/docker/compose.yaml" up -d commander locomotion_bridge mission_server station_bridge
bash "$REPO/tools/dev/container_deps.sh"
for _ in $(seq 1 30); do
  docker logs docker-locomotion_bridge-1 2>&1 | grep -qa "M20 answering" && break
  docker logs docker-locomotion_bridge-1 2>&1 | grep -qa "no status from the M20" && { echo "ERROR: the bridge cannot reach the robot:"; docker logs --tail 5 docker-locomotion_bridge-1; exit 1; }
  sleep 1
done
docker logs docker-locomotion_bridge-1 2>&1 | grep -qa "M20 answering" || { echo "ERROR: bridge did not report the robot in 30 s: docker logs docker-locomotion_bridge-1"; exit 1; }
echo "  $(docker logs docker-locomotion_bridge-1 2>&1 | grep -a 'M20 answering' | tail -1)"

echo "[2/6] staging config into $C:/cfg"
docker exec "$C" bash -lc "mkdir -p /cfg/out /cfg/maps && rm -f /cfg/maps/site.*"
docker cp "$MAP" "$C:/cfg/maps/site.yaml"
docker cp "${MAP%.yaml}.pgm" "$C:/cfg/maps/$(basename "${MAP%.yaml}").pgm"
docker cp "$REPO/src/m20_navigation/config/nav2_params.yaml" "$C:/cfg/nav2_params.yaml"
docker cp "$REPO/src/m20_navigation/config/pointcloud_to_laserscan.yaml" "$C:/cfg/pointcloud_to_laserscan.yaml"

echo "[3/6] map_server: the robot's map on /map (frame map = the vendor localization frame)"
docker exec -d "$C" bash -lc "$SRC; ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/cfg/maps/site.yaml -p use_sim_time:=false >/cfg/out/map_server.log 2>&1"
docker exec -d "$C" bash -lc "$SRC; ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_map -p autostart:=true -p node_names:=\"['map_server']\" >/cfg/out/map_lifecycle.log 2>&1"

if [ "$NAV" = nav2 ]; then
  echo "[4/6] Nav2 on the vendor map; lidar: RoboSense Airy x2 via rslidar_sdk -> /scan_front + /scan_rear"
  # Lidar mount transforms measured on the robot (floor fits, 2026-10-15): both units pitched -90 deg.
  docker exec -d "$C" bash -lc "$SRC; ros2 run tf2_ros static_transform_publisher --x 0.32028 --y 0 --z -0.013 --roll 0 --pitch -1.57079 --yaw -3.14159 --frame-id base_link --child-frame-id rslidar_front"
  docker exec -d "$C" bash -lc "$SRC; ros2 run tf2_ros static_transform_publisher --x -0.32028 --y 0 --z -0.013 --roll 0 --pitch -1.57079 --yaw 0 --frame-id base_link --child-frame-id rslidar_rear"
  for side in front rear; do
    docker exec -d "$C" bash -lc "$SRC; ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=laserscan_$side -r cloud_in:=/rslidar_points_$side -r scan:=/scan_$side --params-file /cfg/pointcloud_to_laserscan.yaml >/cfg/out/laserscan_$side.log 2>&1"
  done
  docker exec -i "$C" python3 - <<'PYEOF'
import yaml
p = "/cfg/nav2_params.yaml"
cfg = yaml.safe_load(open(p))
g = cfg["global_costmap"]["global_costmap"]["ros__parameters"]
g["rolling_window"] = False; g.pop("width", None); g.pop("height", None); g["track_unknown_space"] = True
g["plugins"] = ["static_layer", "obstacle_layer", "inflation_layer"]
g["static_layer"] = {"plugin": "nav2_costmap_2d::StaticLayer", "map_subscribe_transient_local": True, "map_topic": "/map"}
for k in ("global_costmap", "local_costmap"):
    ob = cfg[k][k]["ros__parameters"]["obstacle_layer"]
    ob["observation_sources"] = "scan_front scan_rear"
    for side in ("front", "rear"):
        ob["scan_" + side] = {"topic": "/scan_" + side, "data_type": "LaserScan", "marking": True, "clearing": True,
                              "obstacle_max_range": 8.0, "raytrace_max_range": 9.0}
    ob.pop("scan", None)
yaml.safe_dump(cfg, open(p, "w"), sort_keys=False)
print("  patched: static site map + two lidar scans in both costmaps")
PYEOF
  docker exec -d "$C" bash -lc "$SRC; ros2 launch nav2_bringup navigation_launch.py params_file:=/cfg/nav2_params.yaml use_sim_time:=false >/cfg/out/nav2.log 2>&1"
  for _ in $(seq 1 40); do docker exec "$C" grep -qa "Managed nodes are active" /cfg/out/nav2.log 2>/dev/null && break; sleep 1; done
  docker exec "$C" grep -qa "Managed nodes are active" /cfg/out/nav2.log || { echo "ERROR: Nav2 did not become active (docker exec $C tail -30 /cfg/out/nav2.log)"; exit 1; }
  echo "  Nav2 active"
  ROBOT_CLOUDS=0
  for side in front rear; do
    docker exec "$C" bash -lc "$SRC; timeout 8 ros2 topic hz /rslidar_points_$side --window 10 2>&1 | grep -q 'average rate'" && ROBOT_CLOUDS=$((ROBOT_CLOUDS + 1)) || echo "  WARNING: no /rslidar_points_$side (start rslidar_sdk on this laptop; NOS multicast relay enabled?)"
  done
  [ "$ROBOT_CLOUDS" -gt 0 ] || { echo "ERROR: no lidar cloud at all: Nav2 would drive blind"; exit 1; }
else
  echo "[4/6] vendor navigation: the bridge serves /navigate_to_pose (nav_mode 0 Straight, tolerance 0.35 m); no Nav2"
fi

echo "[5/6] arming"
if [ "$ARM" = 1 ]; then
  arm() { docker exec "$C" bash -lc "$SRC; timeout 15 ros2 service call /m20/set_mode m20_msgs/srv/SetMode '{mode: $1, requester: robot_up}'" | grep -c "accepted=True" || true; }
  OK=$(arm 2)
  if [ "$OK" != "1" ]; then arm 0 >/dev/null; sleep 1; OK=$(arm 2); fi
  [ "$OK" = "1" ] || { echo "ERROR: the commander refused ASSISTED (failsafe latched? docker logs $C)"; exit 1; }
  MODE=$(docker exec -i "$C" bash -lc "$SRC; timeout 20 python3 -" <<'PYEOF'
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from m20_msgs.msg import RobotMode
rclpy.init(); n = Node("robot_up_mode_probe"); got = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(RobotMode, "/m20/mode", lambda m: got.append(m), qos)
for _ in range(100):
    rclpy.spin_once(n, timeout_sec=0.1)
    if got: break
print(f"{got[-1].mode} {int(got[-1].armed)}" if got else "none 0")
PYEOF
)
  [ "$MODE" = "2 1" ] || { echo "ERROR: /m20/mode reads '$MODE', expected ASSISTED armed (2 1)"; exit 1; }
  echo "  armed ASSISTED (verified on /m20/mode)"
else
  echo "  --no-arm: commander left disarmed"
fi

if [ -n "$OBJECT_MAP" ]; then
  echo "[6/6] NL navigation (Ollama + object map + OWLv2 text encoder)"
  bash "$REPO/tools/nlnav/ollama_up.sh"
  bash "$REPO/tools/nlnav/nlnav_up.sh" --object-map "$OBJECT_MAP" --scene "$SCENE" --wait \
    $([ "$NAV" = vendor ] && echo "--no-nav2")
else
  echo "[6/6] no --object-map: NL navigation not started (tools/nlnav/nlnav_up.sh later)"
fi
echo "[robot_up] ready: tools/dev/m20 goto X Y | say \"...\" | status      (Ctrl-C here tears the navigation layer down)"
while sleep 5; do
  docker logs --since 5s docker-locomotion_bridge-1 2>&1 | grep -a "HALT\|ERROR\|error" || true
done
