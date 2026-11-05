#!/usr/bin/env python3
"""Bench tool for the first steps on a real M20: talk to the robot over UDP with no ROS at all.

  python3 tools/robot/m20_probe.py status                 heartbeat + print basic / motion / location / battery for 5 s
  python3 tools/robot/m20_probe.py watch                  the same, continuously (Ctrl-C to stop)
  python3 tools/robot/m20_probe.py mode regular|navigation|assist
  python3 tools/robot/m20_probe.py stand | control | sit | estop
                                                          2/22 Stand, RL Control (takes axis commands), Sit, Soft E-stop
  python3 tools/robot/m20_probe.py gait flat|basic|stair  2/23 (Agile Flat is the one to drive under)
  python3 tools/robot/m20_probe.py drive X Y YAW SECS     stream axis RATIOS at 20 Hz for SECS seconds, then zeros;
                                                          prints the reported LinearX integral and the vendor pose
                                                          before and after, so the ratio -> m/s table can be measured
                                                          against a tape measure (docs/REAL_ROBOT.md, step 2)
  python3 tools/robot/m20_probe.py locate X Y YAW         seed the vendor localization and poll it for 10 s
  --ip 10.21.31.103 --port 30000 override the robot address.

Safety: `drive` refuses ratios above 0.3 unless --unsafe is given; someone holds the pendant's e-stop; the
robot must be standing in RL Control, in Regular usage mode, off the dock, with 2 m clear in every direction.
This is the ONLY UDP client that may run: the pendant app and the bridge both displace it (E006).
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src", "m20_locomotion_bridge"))
from m20_locomotion_bridge import m20_udp as p   # noqa: E402

MODES = {"regular": p.MODE_REGULAR, "navigation": p.MODE_NAVIGATION, "assist": p.MODE_ASSIST}
STATES = {"stand": p.MS_STAND, "control": p.MS_RL_CONTROL, "sit": p.MS_SIT, "estop": p.MS_SOFT_ESTOP}
GAITS = {"flat": p.GAIT_FLAT_AGILE, "basic": p.GAIT_BASIC, "stair": p.GAIT_STAIR_AGILE}
STATE_NAMES = {0: "Idle", 1: "Stand", 2: "SoftEStop", 3: "Damping", 4: "Sit", 6: "Standard", 8: "Agile", 17: "RLControl"}


def snapshot(link):
    b = link.latest(p.STATUS_BASIC, 3.0)
    m = link.latest(p.STATUS_MOTION, 1.0)
    loc = link.latest(p.STATUS_NAV, 3.0)
    dev = link.latest(p.STATUS_DEVICE, 3.0)
    out = []
    if b:
        bs = p.parse_basic(b)
        out.append(f"state={STATE_NAMES.get(bs.motion_state, bs.motion_state)} gait=0x{bs.gait:04x} mode={bs.usage_mode} "
                   f"charge={bs.charge} HES={int(bs.hard_estop)}")
    if m:
        ms = p.parse_motion(m)
        out.append(f"vx={ms.vx:+.3f} vy={ms.vy:+.3f} wz={ms.wz:+.3f} rpy=({ms.roll:+.2f},{ms.pitch:+.2f},{ms.yaw:+.2f}) h={ms.height:.2f}")
    if loc:
        ls = p.parse_location(loc)
        out.append(f"map=({ls.x:+.2f},{ls.y:+.2f},{ls.yaw:+.2f}) loc={'ok' if ls.ok else 'LOST'} nav={ls.nav_status}/0x{ls.nav_error:x}")
    if dev:
        bat = p.parse_battery(dev)
        out.append(f"battery={bat:.0f}%" if bat is not None else "battery=?")
    return " | ".join(out) if out else "no status yet (heartbeats reach the robot? same wire? firewall?)"


def wait_for(link, code, secs, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        if link.latest(code, 3.0) is not None:
            return True
        time.sleep(0.1)
    print(f"ERROR: no {what} within {secs:.0f} s: is the robot on and reachable at {link.addr[0]}:{link.addr[1]}?")
    return False


def cmd_status(link, a):
    if not wait_for(link, p.STATUS_BASIC, 5.0, "basic status"):
        return 1
    t0 = time.monotonic()
    while time.monotonic() - t0 < (1e9 if a.cmd == "watch" else 5.0):
        print(snapshot(link), flush=True)
        time.sleep(1.0)
    return 0


def cmd_set(link, a):
    if not wait_for(link, p.STATUS_BASIC, 5.0, "basic status"):
        return 1
    if a.cmd == "mode":
        link.send_msg(p.usage_mode(MODES[a.arg]))
    elif a.cmd == "gait":
        link.send_msg(p.gait(GAITS[a.arg]))
    else:
        link.send_msg(p.motion_state(STATES[a.cmd]))
    for _ in range(8):
        time.sleep(0.5)
        print(snapshot(link), flush=True)
    ack = link.last_ack(p.USAGE_MODE if a.cmd == "mode" else p.GAIT if a.cmd == "gait" else p.MOTION_STATE, 10.0)
    print("ack:", ack)
    return 0


def cmd_drive(link, a):
    rx, ry, rz = a.x, a.y, a.yaw
    if max(abs(rx), abs(ry), abs(rz)) > 0.3 and not a.unsafe:
        print("refusing ratios above 0.3 without --unsafe (0.3 in X is ~0.6 m/s on a stock M20)")
        return 2
    if not wait_for(link, p.STATUS_MOTION, 5.0, "motion status"):
        return 1
    b = p.parse_basic(link.latest(p.STATUS_BASIC, 3.0) or {"BasicStatus": {"MotionState": -1, "Gait": 0, "ControlUsageMode": -1}})
    if b.motion_state != p.MS_RL_CONTROL or b.usage_mode != p.MODE_REGULAR:
        print(f"refusing: robot must be in RL Control (17) and Regular mode (0); it is state={b.motion_state} mode={b.usage_mode}")
        return 2
    loc0 = link.latest(p.STATUS_NAV, 3.0)
    print("before:", snapshot(link))
    dist = 0.0
    t_last = time.monotonic()
    t_end = t_last + a.secs
    bias = 0.0
    samples = [p.parse_motion(link.latest(p.STATUS_MOTION, 1.0)).vx for _ in range(5) if link.latest(p.STATUS_MOTION, 1.0)]
    if samples:
        bias = sum(samples) / len(samples)
    while time.monotonic() < t_end:
        link.send_msg(p.axis(rx, ry, rz))
        time.sleep(0.05)                                            # 20 Hz
        m = link.latest(p.STATUS_MOTION, 1.0)
        now = time.monotonic()
        if m:
            dist += (p.parse_motion(m).vx - bias) * (now - t_last)
        t_last = now
    for _ in range(3):
        link.send_msg(p.axis(0.0, 0.0, 0.0))
        time.sleep(0.05)
    time.sleep(1.5)
    print("after: ", snapshot(link))
    print(f"integrated LinearX (bias {bias:+.3f} removed): {dist:.2f} m over {a.secs:.1f} s at ratio {rx:+.2f}")
    loc1 = link.latest(p.STATUS_NAV, 3.0)
    if loc0 and loc1:
        l0, l1 = p.parse_location(loc0), p.parse_location(loc1)
        d = math.hypot(l1.x - l0.x, l1.y - l0.y)
        print(f"vendor pose moved {d:.2f} m, turned {math.degrees(p.wrap(l1.yaw - l0.yaw)):+.1f} deg")
        if rx and d:
            print(f"=> vx_max estimate {d / a.secs / abs(rx):.2f} m/s at ratio 1.0 (tape-measure the floor to confirm)")
    return 0


def cmd_locate(link, a):
    link.send_msg(p.location_init(a.x, a.y, a.yaw))
    for _ in range(10):
        time.sleep(1.0)
        link.send(p.LOC_STATE)
        print(snapshot(link), flush=True)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=p.ROBOT_IP)
    ap.add_argument("--port", type=int, default=p.ROBOT_PORT)
    ap.add_argument("--unsafe", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for c in ("status", "watch", "stand", "control", "sit", "estop"):
        sub.add_parser(c)
    sub.add_parser("mode").add_argument("arg", choices=MODES)
    sub.add_parser("gait").add_argument("arg", choices=GAITS)
    d = sub.add_parser("drive")
    for n in ("x", "y", "yaw", "secs"):
        d.add_argument(n, type=float)
    loc = sub.add_parser("locate")
    for n in ("x", "y", "yaw"):
        loc.add_argument(n, type=float)
    a = ap.parse_args()
    link = p.Link(a.ip, a.port)
    link.start()
    try:
        if a.cmd in ("status", "watch"):
            return cmd_status(link, a)
        if a.cmd in ("mode", "gait", "stand", "control", "sit", "estop"):
            return cmd_set(link, a)
        if a.cmd == "drive":
            return cmd_drive(link, a)
        return cmd_locate(link, a)
    except KeyboardInterrupt:
        for _ in range(3):
            link.send_msg(p.axis(0.0, 0.0, 0.0))
            time.sleep(0.05)
        return 130
    finally:
        link.stop()


if __name__ == "__main__":
    sys.exit(main())
