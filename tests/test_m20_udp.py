"""The M20 UDP protocol module, checked against the vendor manual's framing and the measured numbers."""
import json
import math
import struct

import pytest

from m20_locomotion_bridge import m20_udp as p


def test_frame_header_matches_the_manual():
    f = p.encode(7, p.HEARTBEAT, when="2023-01-01 00:00:00")
    assert f[:4] == b"\xeb\x91\xeb\x90"                       # sync characters, fixed
    length, msg_id = struct.unpack("<HH", f[4:8])
    assert msg_id == 7 and f[8] == 0x01 and f[9:16] == b"\x00" * 7
    body = json.loads(f[16:16 + length])
    assert body == {"PatrolDevice": {"Type": 100, "Command": 100, "Time": "2023-01-01 00:00:00", "Items": {}}}
    assert len(f) == 16 + length


def test_decode_roundtrip_and_msg_id_wraps():
    f = p.encode(0x1FFFF, p.AXIS, {"X": 0.5})
    msg_id, code, items = p.decode(f)
    assert (msg_id, code, items) == (0xFFFF, p.AXIS, {"X": 0.5})


@pytest.mark.parametrize("bad", [b"", b"\x00" * 15, b"\xeb\x91\xeb\x91" + b"\x00" * 12,
                                 b"\xeb\x91\xeb\x90" + struct.pack("<HH", 50, 1) + b"\x01" + b"\x00" * 7 + b"{}"])
def test_decode_rejects_malformed_frames(bad):
    with pytest.raises(ValueError):
        p.decode(bad)


def test_ack_is_told_apart_by_content_not_code():
    ack = {"ErrorCode": 0, "ErrorMessage": "ok"}
    push = {"BasicStatus": {"MotionState": 17, "Gait": 0x3002, "ControlUsageMode": 0}}
    assert p.is_ack(ack) and p.is_ack({}) and not p.is_ack(push)


def test_axis_clamps_ratios_and_zeroes_unused_axes():
    code, items = p.axis(1.7, -3.0, 0.25)
    assert code == (2, 21)
    assert items == {"X": 1.0, "Y": -1.0, "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.25}


def test_nav_task_payload_fields():
    code, t = p.nav_task(1.5, -2.0, math.pi / 2, nav_mode=1, obstacle_avoidance=False)
    assert code == (1003, 1)
    assert t["PosX"] == 1.5 and t["PosY"] == -2.0 and t["AngleYaw"] == pytest.approx(math.pi / 2)
    assert t["Gait"] == 0x3002 and t["Speed"] == 1 and t["NavMode"] == 1 and t["ObsMode"] == 1
    assert t["PointInfo"] == 1                                  # a task point reports Complete; a transition reports Idle


def test_parsers_read_the_push_formats():
    m = p.parse_motion({"MotionStatus": {"Roll": 0.01, "Pitch": -0.02, "Yaw": 1.5, "OmegaZ": 0.3,
                                         "LinearX": 0.42, "LinearY": -0.1, "Height": 0.45, "Payload": 0, "RemainMile": 3}})
    assert (m.vx, m.vy, m.wz, m.yaw, m.height) == (0.42, -0.1, 0.3, 1.5, 0.45)
    b = p.parse_basic({"BasicStatus": {"MotionState": 17, "Gait": 12290, "Charge": 0, "HES": 1, "ControlUsageMode": 1}})
    assert b.motion_state == p.MS_RL_CONTROL and b.gait == p.GAIT_FLAT_AGILE and b.hard_estop and b.usage_mode == p.MODE_NAVIGATION
    loc = p.parse_location({"NavStatus": {"Status": 4, "ErrorCode": 0},
                            "LocationStatus": {"PosX": 3.0, "PosY": 4.0, "Yaw": 0.5, "Location": 0, "Confidence": 0.9}})
    assert loc.ok and loc.nav_status == 4 and (loc.x, loc.y, loc.yaw) == (3.0, 4.0, 0.5)
    lost = p.parse_location({"LocationStatus": {"PosX": 0, "PosY": 0, "Yaw": 0, "Location": 1, "Confidence": 0.85}})
    assert not lost.ok                                           # confidence 0.85 while lost: never gate on it
    assert p.parse_battery({"BatteryStatus": {"BatteryLevelLeft": 70, "BatteryLevelRight": 74}}) == 72.0
    assert p.parse_battery({}) is None


def test_speed_table_measured_defaults_and_dead_zones():
    t = p.SpeedTable()
    rx, ry, rz = t.ratios(0.5, 0.0, 0.0)
    assert rx == pytest.approx(0.5 / 2.1) and ry == 0.0 and rz == 0.0
    assert t.ratios(5.0, 0.0, 0.0)[0] == 1.0                     # clamped at the maximum
    assert t.ratios(0.0, 0.0, -0.05)[2] == pytest.approx(-0.15 / 1.8)   # lifted to the yaw dead zone, sign kept
    assert t.ratios(0.0, 0.05, 0.0)[1] == pytest.approx(0.16 / 0.62)
    assert t.ratios(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)            # zero stays zero: a stop is a stop


def test_pose_compose_invert_are_inverses():
    a = (1.0, 2.0, 0.7)
    x, y, th = p.compose(a, p.invert(a))
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(th) < 1e-9
    assert p.compose((1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0)) == pytest.approx((1.0, 1.0, math.pi / 2))


def _motion(vx=0.0, vy=0.0, yaw=0.0):
    return p.MotionStatus(vx, vy, 0.0, 0.0, 0.0, yaw, 0.45)


def test_dead_reckoner_integrates_in_the_odom_frame():
    d = p.DeadReckoner()
    d.step(_motion(yaw=1.0), 0.0)                              # first sample defines the origin: yaw -> 0
    for i in range(1, 11):
        d.step(_motion(vx=0.5, yaw=1.0), 0.1 * i)                # 1 s at 0.5 m/s straight
    assert d.pose == pytest.approx((0.5, 0.0, 0.0), abs=1e-9)
    for i in range(11, 21):
        d.step(_motion(vx=0.5, yaw=1.0 + math.pi / 2), 0.1 * i)  # turned left, another 1 s
    x, y, yaw = d.pose
    assert x == pytest.approx(0.5) and y == pytest.approx(0.5) and yaw == pytest.approx(math.pi / 2)


def test_dead_reckoner_ignores_the_rest_bias_and_dropouts():
    d = p.DeadReckoner(v_deadband=0.06)
    d.step(_motion(), 0.0)
    d.step(_motion(vx=0.05), 0.5)                                # the ~0.05 m/s LinearX bias while standing
    assert d.pose[0] == 0.0
    d.step(_motion(vx=1.0), 3.0)                                 # 2.5 s gap: a dropout, not 2.5 m of motion
    assert d.pose[0] == 0.0
    d.step(_motion(vx=1.0), 3.1)
    assert d.pose[0] == pytest.approx(0.1)


def test_map_correction_reproduces_the_vendor_pose():
    c = p.MapCorrection()
    odom_base = (2.0, 0.5, 0.3)
    map_base = (10.0, -4.0, 1.8)
    c.update(map_base, odom_base)
    assert c.valid and c.map_pose(odom_base) == pytest.approx(map_base)
    # the robot moves 1 m forward in odom: the map pose moves 1 m along its own heading
    moved = p.compose(odom_base, (1.0, 0.0, 0.0))
    mx, my, _ = c.map_pose(moved)
    assert math.hypot(mx - map_base[0], my - map_base[1]) == pytest.approx(1.0)


def test_parse_basic_tolerates_a_sender_without_gait_or_usage_mode():
    # the protocol emulator sends only MotionState / Charge / HES / ControlUsageMode; a bare MotionState is enough
    b = p.parse_basic({"BasicStatus": {"MotionState": 17, "Charge": 0, "HES": 0, "ControlUsageMode": 0}})
    assert (b.motion_state, b.gait, b.charge, b.hard_estop, b.usage_mode) == (17, 0, 0, False, 0)
    b = p.parse_basic({"BasicStatus": {"MotionState": 4}})
    assert (b.motion_state, b.gait, b.usage_mode) == (4, 0, 0)
