#!/usr/bin/env bash
# The lidar stage on armdog: one container (m20_lidar, image docker/Dockerfile.rslidar) that runs the RoboSense driver,
# the two scan converters and the two mount transforms, so the big clouds (~28k / ~47k points at 10 Hz) never leave
# the container and only the small /scan_front and /scan_rear cross to the Nav2 stack.
#
#   bash tools/lidar/lidar_up.sh --check            are the relayed packets arriving? (tools/lidar/rslidar_check.py, no ROS)
#   bash tools/lidar/lidar_up.sh --start            start the container, the converters and the TFs; verify clouds + scans
#   bash tools/lidar/lidar_up.sh --stop
#   bash tools/lidar/lidar_up.sh --rates [secs]     cloud and scan rates and loss over `secs` (default 60): the walk's step 1
#
# Transport: the driver and the converters talk Fast DDS with shared memory (RMW_IMPLEMENTATION=rmw_fastrtps_cpp,
# --ipc host). With the default UDP transport in a container 14 % (front) and 27 % (rear) of clouds were lost
# between the driver and a subscriber (measured 2026-10-30); shared memory is the fix for that. The scans reach the
# Cyclone-based stack over the domain's UDP as any RTPS topic does. LIDAR_RMW overrides the choice if the numbers say so.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
C=m20_lidar
IMG="${LIDAR_IMAGE:-m20_lidar:latest}"
RMW="${LIDAR_RMW:-rmw_fastrtps_cpp}"
DOM="${ROS_DOMAIN_ID:-42}"
HOST_ADDR="${LIDAR_HOST:-10.21.31.50}"
case "${1:-}" in
  --check)
    python3 "$REPO/tools/lidar/rslidar_check.py" --host "$HOST_ADDR" --secs 3 ;;
  --start)
    docker image inspect "$IMG" >/dev/null 2>&1 || { echo "lidar_up: image $IMG missing: docker build -f docker/Dockerfile.rslidar -t $IMG ."; exit 1; }
    python3 "$REPO/tools/lidar/rslidar_check.py" --host "$HOST_ADDR" --secs 2 || exit 1
    docker rm -f $C >/dev/null 2>&1 || true
    docker run -d --name $C --network host --ipc host \
      -e ROS_DOMAIN_ID="$DOM" -e RMW_IMPLEMENTATION="$RMW" \
      -v "$REPO/tools/lidar/rslidar_config.yaml:/cfg/rslidar_config.yaml:ro" \
      -v "$REPO/src/m20_navigation/config/pointcloud_to_laserscan.yaml:/cfg/pointcloud_to_laserscan.yaml:ro" \
      "$IMG" bash -lc "ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/cfg/rslidar_config.yaml" >/dev/null
    sleep 4
    docker ps --format '{{.Names}}' | grep -qx $C || { echo "lidar_up: the driver exited:"; docker logs --tail 20 $C; exit 1; }
    RMW_IMPLEMENTATION="$RMW" ROS_DOMAIN_ID="$DOM" bash "$REPO/tools/nav/lidar_stage.sh" $C --tf
    sleep 3
    RMW_IMPLEMENTATION="$RMW" ROS_DOMAIN_ID="$DOM" bash "$REPO/tools/nav/lidar_stage.sh" $C --check ;;
  --stop)
    docker rm -f $C >/dev/null 2>&1 && echo "lidar_up: $C stopped" || echo "lidar_up: $C was not running" ;;
  --rates)
    # per topic over N seconds: messages received, the rate the stamps imply, and the loss between the two
    docker exec -i -e SECS="${2:-60}" $C bash -lc "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=$DOM RMW_IMPLEMENTATION=$RMW; python3 -" <<'PY'
import os, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
secs = float(os.environ["SECS"])
rclpy.init(); n = rclpy.create_node("lidar_rates"); got = {}
def cb(topic):
    def f(m):
        st = got.setdefault(topic, [0, None, None])
        st[0] += 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        st[1] = t if st[1] is None else st[1]; st[2] = t
    return f
for side in ("front", "rear"):
    n.create_subscription(PointCloud2, f"/rslidar_points_{side}", cb(f"cloud_{side}"), qos_profile_sensor_data)
    n.create_subscription(LaserScan, f"/scan_{side}", cb(f"scan_{side}"), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.2)
for k in sorted(got):
    cnt, t_first, t_last = got[k]
    span = (t_last - t_first) if t_first is not None and t_last > t_first else 0.0
    expected = span * 10.0 + 1                       # the lidars sweep at 10 Hz; the converters follow them
    loss = max(0.0, 1.0 - cnt / expected) if span else float("nan")
    print(f"{k:12s} {cnt:5d} msgs in {secs:.0f} s = {cnt / secs:5.2f} Hz, stamp span {span:6.1f} s, loss {loss * 100:5.1f} %")
missing = [k for k in ("cloud_front", "cloud_rear", "scan_front", "scan_rear") if k not in got]
print("MISSING: " + " ".join(missing) if missing else "all four topics seen")
PY
    ;;
  *) sed -n '2,10p' "$0"; exit 2 ;;
esac
