#!/usr/bin/env python3
"""Are the two RoboSense Airy lidars reaching this computer? Host Python, no ROS, no root.

The lidars do not send to armdog directly: the robot's navigation computer (NOS, 10.21.31.106) relays their raw
packets as multicast on the robot subnet, front 224.10.10.201 (points 6691, device info 7781), rear 224.10.10.202
(6692, 7782). This joins the four groups on the given host address for a few seconds and prints what arrived.
Expected (measured 2026-10-30): about 2,238 packets/s and 2.79 MB/s of points per lidar, device info 1/s, source
10.21.31.106. Silence means NOS's `multicast-relay.service` is off (it ships off, and an OTA turns it off): the robot
team has to enable it; no amount of driver configuration helps.

  python3 tools/lidar/rslidar_check.py [--host 10.21.31.50] [--secs 3]
Exit 0 when both lidars send points, 1 otherwise. The host address must be THIS machine's address on the robot
subnet; binding to the wrong interface yields silence with no error.
"""
import argparse
import select
import socket
import struct
import sys
import time

LIDARS = {"front": ("224.10.10.201", 6691, 7781), "rear": ("224.10.10.202", 6692, 7782)}


def join(group, port, host):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton(host)))
    s.setblocking(False)
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="10.21.31.50", help="this machine's address on the robot subnet")
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()
    socks = {}
    for name, (group, msop, difop) in LIDARS.items():
        socks[join(group, msop, a.host)] = (name, "points")
        socks[join(group, difop, a.host)] = (name, "info")
    stats = {k: [0, 0, set()] for k in socks.values()}          # packets, bytes, sources
    t_end = time.monotonic() + a.secs
    while time.monotonic() < t_end:
        ready, _, _ = select.select(list(socks), [], [], 0.2)
        for s in ready:
            data, (src, _) = s.recvfrom(65535)
            st = stats[socks[s]]
            st[0] += 1; st[1] += len(data); st[2].add(src)
    ok = True
    for name in LIDARS:
        pk, by, src = stats[(name, "points")]
        ik, _, _ = stats[(name, "info")]
        print(f"{name}: points {pk / a.secs:7.0f} pkt/s {by / a.secs / 1e6:5.2f} MB/s, device info {ik / a.secs:4.1f}/s, "
              f"source {', '.join(sorted(src)) or 'none'}")
        ok = ok and pk > 0
    if not ok:
        print(f"no packets from at least one lidar on {a.host}: is this the address on the robot subnet, and is NOS's "
              "multicast-relay.service enabled (off after an OTA)?")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
