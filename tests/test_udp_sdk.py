"""M20UdpSDK (the real-robot backend) against a fake Link: the position report is published once per NEW report
with its receive stamp, axis frames are gated on Regular usage mode, stop() is three zero frames then silence."""
import time

import pytest
import rclpy
from rclpy.node import Node

import tf2_ros
from m20_locomotion_bridge import m20_udp as p
from m20_locomotion_bridge.bridge_node import M20UdpSDK


class FakeLink:
    def __init__(self):
        self.sent, self.entries, self.addr = [], {}, ("127.0.0.1", 30000)
    def start(self): pass
    def send_msg(self, msg): self.sent.append(msg)
    def latest_entry(self, code, max_age=2.0): return self.entries.get(code)
    def latest(self, code, max_age=2.0):
        e = self.entries.get(code); return None if e is None else e.items
    def push(self, code, items, t=None):
        self.entries[code] = p._Entry(time.monotonic() if t is None else t, items)


def basic(mode=p.MODE_REGULAR):
    return {"BasicStatus": {"MotionState": 17, "Gait": 12290, "Charge": 0, "HES": 0, "ControlUsageMode": mode}}

def location(x, y, yaw=0.0):
    return {"LocationStatus": {"PosX": x, "PosY": y, "Yaw": yaw, "Location": 0}}

def motion(vx=0.0):
    return {"MotionStatus": {"LinearX": vx, "LinearY": 0.0, "OmegaZ": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}}

def sdk():
    s = M20UdpSDK(Node("t"))
    s.link = FakeLink()
    tf2_ros.SENT.clear()
    return s

def axis_frames(link):
    return [m for m in link.sent if m[0] == p.AXIS]


def test_vendor_pose_once_per_new_report_with_the_receive_stamp():
    s = sdk()
    rclpy.set_time(100.0)
    s.link.push(p.STATUS_BASIC, basic())
    s.link.push(p.STATUS_MOTION, motion())
    s.link.push(p.STATUS_NAV, location(1.0, 2.0), t=time.monotonic() - 0.3)    # received 0.3 s ago
    s._state(); s._state(); s._state()                                            # the 10 Hz loop reads the same cache
    poses = rclpy.PUBLISHED["/m20/vendor_pose"]
    assert len(poses) == 1                                                        # one message per report, not per tick
    stamp = poses[0].header.stamp.sec + poses[0].header.stamp.nanosec * 1e-9
    assert stamp == pytest.approx(99.7, abs=0.05)                                 # now minus how long ago it arrived
    assert poses[0].pose.position.x == 1.0 and poses[0].header.frame_id == "map"
    s.link.push(p.STATUS_NAV, location(1.5, 2.0))                                 # a new report
    s._state()
    assert len(poses) == 2 and poses[1].pose.position.x == 1.5
    assert not [t for t in tf2_ros.SENT if t.header.frame_id == "map"]            # no map->odom from the bridge any more
    assert [t.child_frame_id for t in tf2_ros.SENT] == ["base_link"] * 4          # odom->base_link every tick


def test_report_delay_parameter_moves_the_stamp_back():
    s = sdk()
    s.delay = 0.1
    rclpy.set_time(50.0)
    s.link.push(p.STATUS_BASIC, basic()); s.link.push(p.STATUS_MOTION, motion())
    s.link.push(p.STATUS_NAV, location(0.0, 0.0), t=time.monotonic())
    s._state()
    m = rclpy.PUBLISHED["/m20/vendor_pose"][0]
    assert m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 == pytest.approx(49.9, abs=0.05)


def test_no_motion_status_fallback_publishes_odom_from_the_report():
    s = sdk()
    s.link.push(p.STATUS_BASIC, basic())
    s.link.push(p.STATUS_NAV, location(3.0, 4.0, 0.5))
    s._state()
    o = rclpy.PUBLISHED["/odom"][0]
    assert (o.pose.pose.position.x, o.pose.pose.position.y) == (3.0, 4.0)
    assert len(rclpy.PUBLISHED["/m20/vendor_pose"]) == 1


def test_axis_frames_only_in_regular_usage_mode():
    s = sdk()
    s.link.push(p.STATUS_BASIC, basic(mode=p.MODE_NAVIGATION))
    s._state()
    s.send_velocity(0.3, 0.0, 0.0)
    s._stream(); s._stream()
    assert axis_frames(s.link) == []                                              # withheld: Navigation mode ignores them
    s.link.push(p.STATUS_BASIC, basic(mode=p.MODE_REGULAR))
    s._state(); s._stream()
    assert len(axis_frames(s.link)) == 1
    s.request_regular()
    assert s.link.sent[-1] == p.usage_mode(p.MODE_REGULAR)


def test_stop_is_three_zero_frames_then_silence():
    s = sdk()
    s.link.push(p.STATUS_BASIC, basic()); s._state()
    s.send_velocity(0.3, 0.0, 0.0); s._stream()
    s.stop()
    frames = axis_frames(s.link)
    assert len(frames) == 4 and all(f[1]["X"] == 0 and f[1]["Y"] == 0 and f[1]["Yaw"] == 0 for f in frames[1:])
    for _ in range(25):                                                           # 1.25 s of the 20 Hz timer
        s._stream()
    assert len(axis_frames(s.link)) == 4                                          # nothing more: the pendant gets the robot back
