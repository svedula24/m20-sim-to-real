#!/usr/bin/env bash
# The camera path without new hardware: the vendor fisheye (RTSP) as a pinhole /camera/image_raw + /camera/camera_info,
# and /camera/depth projected from the front lidar cloud (needs rslidar_sdk running and the lidar TF from robot_up.sh).
#   bash tools/camera/camera_up.sh [--url rtsp://10.21.31.103:8554/video1] [--no-depth]
# Runs both nodes in the m20_nlnav image (cv2 + sensor_msgs_py) on the host network; `docker rm -f m20_camera` stops them.
# With an RGB-D camera instead, run its vendor node with the same three topic names and skip this script.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
URL="${CAM_URL:-rtsp://10.21.31.103:8554/video1}"; DEPTH=1
while [ $# -gt 0 ]; do
  case "$1" in
    --url) URL="$2"; shift ;;
    --no-depth) DEPTH=0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
CMD="source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /camera/fisheye_camera_node.py"
[ "$DEPTH" = 1 ] && CMD+=" & python3 /camera/lidar_depth_node.py"
CMD+="; wait"
docker rm -f m20_camera >/dev/null 2>&1 || true
docker run -d --name m20_camera --network host --ipc host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e PYTHONUNBUFFERED=1 \
  -e CAM_URL="$URL" -e CAM_HZ="${CAM_HZ:-5}" -e LIDAR_TOPIC="${LIDAR_TOPIC:-/rslidar_points_front}" \
  -v "$REPO/tools/camera":/camera:ro m20_nlnav:latest bash -lc "$CMD" >/dev/null
sleep 5
docker ps --format '{{.Names}}' | grep -q '^m20_camera$' || { echo "ERROR: camera container exited:"; docker logs m20_camera | tail -20; exit 1; }
docker logs m20_camera 2>&1 | grep -a "streaming\|depth for\|ERROR\|cannot open" | tail -3
echo "[camera_up] check: ros2 topic hz /camera/image_raw  (and /camera/depth)"
