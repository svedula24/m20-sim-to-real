"""Deep Robotics Lynx M20 inspection protocol over UDP: the only transport this stack uses to move the robot.

Pure Python (no ROS): framing, the message set we use, the measured ratio-to-speed table, and dead reckoning from
the robot's reported body velocity (the map->odom correction from the robot's own localization lives in
tools/nav/map_tf_adapter.py, shared with the sim repo). Everything here has unit tests (tests/test_m20_udp.py); the
socket class is exercised with a fake socket there and for real only on the robot.

Facts, from the vendor's Software Development Guide (V1.2.1 / the 2025-08 beta) and from runs on a real M20
(firmware 1.2.0.1), see docs/REAL_ROBOT.md:
  * The robot is a UDP server at 10.21.31.103:30000. Frames are a 16-byte header (sync EB 91 EB 90, ASDU length
    uint16 LE, message id uint16 LE, format byte 0x01 = JSON, 7 reserved zero bytes) followed by one JSON ASDU
    {"PatrolDevice": {"Type", "Command", "Time", "Items"}}.
  * Every request is acknowledged with the SAME Type/Command carrying only ErrorCode/ErrorMessage/Code; status
    pushes use the same Type/Command with real Items. Filter ACKs by content (is_ack), never by code.
  * Pushes go to whoever sends heartbeats (100/100, at least 1 Hz; we send 2 Hz): 1002/6 basic status 2 Hz,
    1002/4 motion status 10 Hz, 1002/5 device status 2 Hz, 1002/7 nav + location status 1 Hz (re-polled to ~2 Hz).
  * Axis command 2/21: X, Y, Yaw as ratios in [-1, 1] of the maximum speed, Regular usage mode only, 20 Hz
    recommended; the robot brakes ~0.5 s after the last frame. Within 2 s only ONE client may send axis commands
    (error E006): the pendant app and this bridge cannot both drive.
  * Measured at ratio 1.0 in the Agile Flat gait (2026-09-29): X 2.1 m/s, Y 0.62 m/s, Yaw 1.8 rad/s. Reported
    LinearX has a ~0.05 m/s bias while standing still.
"""
from __future__ import annotations

import json
import logging
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

ROBOT_IP = "10.21.31.103"
ROBOT_PORT = 30000

_MAGIC = b"\xeb\x91\xeb\x90"
_HEADER = struct.Struct("<4sHHB7s")      # sync, ASDU length, message id, format, reserved
HEADER_LEN = _HEADER.size               # 16
FORMAT_JSON = 0x01

# (Type, Command) codes
HEARTBEAT = (100, 100)
USAGE_MODE = (1101, 5)
LIGHTS = (1101, 2)
AUTO_SLEEP = (1101, 6)
MOTION_STATE = (2, 22)
GAIT = (2, 23)
AXIS = (2, 21)
CHARGE = (2, 24)
STATUS_ERRORS = (1002, 3)
STATUS_MOTION = (1002, 4)
STATUS_DEVICE = (1002, 5)
STATUS_BASIC = (1002, 6)
STATUS_NAV = (1002, 7)
NAV_TASK = (1003, 1)
NAV_CANCEL = (1004, 1)
LOC_POSE = (1007, 2)
LOC_INIT = (2101, 1)
LOC_STATE = (2002, 1)
POLLS = (STATUS_ERRORS, STATUS_DEVICE, STATUS_BASIC, STATUS_NAV)   # re-polled with every heartbeat

# Usage modes (1101/5 "Mode"; BasicStatus.ControlUsageMode)
MODE_REGULAR, MODE_NAVIGATION, MODE_ASSIST = 0, 1, 2
# Motion states (2/22 "MotionParam"; BasicStatus.MotionState). 17 = the controller that takes axis commands on
# firmware 1.2.x ("RL Control"); the beta manual's 6/8 (Standard/Agile) are what older firmware reported.
MS_IDLE, MS_STAND, MS_SOFT_ESTOP, MS_DAMPING, MS_SIT, MS_STANDARD, MS_AGILE, MS_RL_CONTROL = 0, 1, 2, 3, 4, 6, 8, 17
# Gaits (2/23 "GaitParam"): agile ones answer faster to velocity and are what the vendor recommends for autonomy
GAIT_BASIC, GAIT_STAIR_STD, GAIT_FLAT_AGILE, GAIT_STAIR_AGILE = 0x1001, 0x1003, 0x3002, 0x3003
LOC_NORMAL, LOC_LOST = 0, 1


# --------------------------------------------------------------------------------------------- framing
def encode(msg_id: int, code: Tuple[int, int], items: Optional[dict] = None, when: Optional[str] = None) -> bytes:
    """One APDU: header + JSON ASDU. `when` is the "Time" field (local time, YYYY-MM-DD HH:MM:SS)."""
    body = {"PatrolDevice": {"Type": code[0], "Command": code[1],
                             "Time": when or time.strftime("%Y-%m-%d %H:%M:%S"), "Items": items or {}}}
    asdu = json.dumps(body, separators=(",", ":")).encode()
    if len(asdu) > 0xFFFF:
        raise ValueError("ASDU longer than 65535 bytes")
    return _HEADER.pack(_MAGIC, len(asdu), msg_id & 0xFFFF, FORMAT_JSON, b"\x00" * 7) + asdu


def decode(datagram: bytes) -> Tuple[int, Tuple[int, int], dict]:
    """-> (message id, (Type, Command), Items). Raises ValueError on a malformed frame."""
    if len(datagram) < HEADER_LEN:
        raise ValueError("short frame")
    sync, length, msg_id, fmt, _ = _HEADER.unpack(datagram[:HEADER_LEN])
    if sync != _MAGIC:
        raise ValueError("bad sync bytes")
    if fmt != FORMAT_JSON:
        raise ValueError(f"unsupported ASDU format 0x{fmt:02x}")
    asdu = datagram[HEADER_LEN:HEADER_LEN + length]
    if len(asdu) != length:
        raise ValueError(f"ASDU length {len(asdu)} != header {length}")
    pd = json.loads(asdu.decode())["PatrolDevice"]
    return msg_id, (int(pd["Type"]), int(pd["Command"])), pd.get("Items") or {}


_ACK_KEYS = {"ErrorCode", "ErrorMessage", "Code"}


def is_ack(items: dict) -> bool:
    """An acknowledgement carries only error fields; a status push has data. Both share Type/Command."""
    return set(items.keys()) <= _ACK_KEYS


# --------------------------------------------------------------------------------------------- messages
def usage_mode(mode: int) -> Tuple[Tuple[int, int], dict]:
    return USAGE_MODE, {"Mode": int(mode)}


def motion_state(param: int) -> Tuple[Tuple[int, int], dict]:
    return MOTION_STATE, {"MotionParam": int(param)}


def gait(param: int) -> Tuple[Tuple[int, int], dict]:
    return GAIT, {"GaitParam": int(param)}


def axis(x: float, y: float, yaw: float) -> Tuple[Tuple[int, int], dict]:
    """Ratios in [-1, 1]; REP-103 signs (x forward, y left, yaw counter-clockwise), verified on the robot."""
    c = lambda v: max(-1.0, min(1.0, float(v)))
    return AXIS, {"X": c(x), "Y": c(y), "Z": 0.0, "Roll": 0.0, "Pitch": 0.0, "Yaw": c(yaw)}


def nav_task(x: float, y: float, yaw: float, nav_mode: int = 0, speed: int = 1, obstacle_avoidance: bool = True,
             gait_param: int = GAIT_FLAT_AGILE) -> Tuple[Tuple[int, int], dict]:
    """Vendor navigation task 1003/1 (Navigation usage mode): drive to (x, y, yaw) in the vendor map frame.
    nav_mode 0 = Straight (direct leg; the accurate one), 1 = Auto (vendor planner; Flat gait at Low speed only).
    speed 0 Normal, 1 Low, 2 High. PointInfo 1 = a task point (0 = transition: reports Idle, not Complete)."""
    return NAV_TASK, {"Value": 0, "MapID": 0, "PosX": float(x), "PosY": float(y), "PosZ": 0.0,
                      "AngleYaw": float(yaw), "PointInfo": 1, "Gait": int(gait_param), "Speed": int(speed),
                      "Manner": 0, "ObsMode": 0 if obstacle_avoidance else 1, "NavMode": int(nav_mode)}


def nav_cancel() -> Tuple[Tuple[int, int], dict]:
    return NAV_CANCEL, {}


def location_init(x: float, y: float, yaw: float) -> Tuple[Tuple[int, int], dict]:
    """Seed the vendor localization (map metres / radians). Check LOC_STATE again ~5 s later: the immediate
    answer is often still "lost"."""
    return LOC_INIT, {"PosX": float(x), "PosY": float(y), "PosZ": 0.0, "Yaw": float(yaw)}


# --------------------------------------------------------------------------------------------- parsing
@dataclass
class MotionStatus:
    vx: float            # m/s, body frame, as reported (LinearX; ~0.05 m/s bias at rest)
    vy: float
    wz: float            # rad/s (OmegaZ)
    roll: float          # rad
    pitch: float
    yaw: float           # rad, the robot's attitude yaw (absolute, IMU-based)
    height: float        # m


def parse_motion(items: dict) -> MotionStatus:
    m = items["MotionStatus"]
    return MotionStatus(float(m["LinearX"]), float(m["LinearY"]), float(m["OmegaZ"]),
                        float(m["Roll"]), float(m["Pitch"]), float(m["Yaw"]), float(m.get("Height", 0.0)))


@dataclass
class BasicStatus:
    motion_state: int
    gait: int
    charge: int
    hard_estop: bool
    usage_mode: int


def parse_basic(items: dict) -> BasicStatus:
    """MotionState is the one field every consumer needs; the rest default when absent. The vendor guide lists
    Gait in BasicStatus (1.3.1.1), but not every sender has it: BrightAI's protocol emulator omits it, and their
    own bridge reads it with .get too (first run against the emulator, 2026-11-05: KeyError 'Gait')."""
    b = items["BasicStatus"]
    return BasicStatus(int(b["MotionState"]), int(b.get("Gait", 0)), int(b.get("Charge", 0)),
                       bool(int(b.get("HES", 0))), int(b.get("ControlUsageMode", 0)))


@dataclass
class LocationStatus:
    x: float
    y: float
    yaw: float
    ok: bool              # Location == 0 (Normal). Do not gate on Confidence: it read 0.85 while lost.
    nav_status: int       # NavStatus.Status (0 idle/none ... 4 complete, 0xff failed)
    nav_error: int        # NavStatus.ErrorCode (0xE01F while idle on firmware 1.2.0.1)


def parse_location(items: dict) -> LocationStatus:
    loc = items["LocationStatus"]
    nav = items.get("NavStatus") or {}
    return LocationStatus(float(loc["PosX"]), float(loc["PosY"]), float(loc["Yaw"]),
                          int(loc.get("Location", LOC_LOST)) == LOC_NORMAL,
                          int(nav.get("Status", 0)), int(nav.get("ErrorCode", 0)))


def parse_battery(items: dict) -> Optional[float]:
    """Battery percent from the device status (1002/5): mean of the two packs, or None if absent."""
    b = items.get("BatteryStatus") or {}
    vals = [float(b[k]) for k in ("BatteryLevelLeft", "BatteryLevelRight") if k in b]
    return sum(vals) / len(vals) if vals else None


# --------------------------------------------------------------------------------------------- speeds
@dataclass
class SpeedTable:
    """Metres per second (radians per second) at ratio 1.0, and the smallest command that moves the robot.

    Defaults are the 2026-09-29 measurement on a real M20 in the Agile Flat gait at 79 % battery: X 2.1 m/s
    (4.64 and 4.71 m in 2.2 s bursts), Y 0.62 m/s, Yaw 1.8 rad/s (about 230 deg). Re-measure with
    tools/robot/m20_probe.py drive on your robot and firmware; put the numbers in the bridge parameters.
    Dead zones: below them the robot does not move, so a non-zero request is lifted to the dead zone (Nav2's
    accel-limited ramps start from the measured rate and can never cross a dead zone on their own).
    """
    vx_max: float = 2.1
    vy_max: float = 0.62
    wz_max: float = 1.8
    vx_dead: float = 0.0
    vy_dead: float = 0.16
    wz_dead: float = 0.15

    def ratios(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        def one(v, vmax, dead):
            if v == 0.0:
                return 0.0
            v = math.copysign(max(abs(v), dead), v)
            return max(-1.0, min(1.0, v / vmax))
        return one(vx, self.vx_max, self.vx_dead), one(vy, self.vy_max, self.vy_dead), one(wz, self.wz_max, self.wz_dead)


# --------------------------------------------------------------------------------------------- poses
Pose2 = Tuple[float, float, float]   # x, y, yaw


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class DeadReckoner:
    """odom -> base_link from the robot's reported body velocity and attitude yaw.

    Heading comes from the reported Yaw (IMU-based, does not drift the way integrated OmegaZ would); position
    integrates LinearX/LinearY rotated into the odom frame. Reported speeds below `v_deadband` are treated as
    zero: LinearX reads ~0.05 m/s while the robot stands still, which would walk the odometry ~3 m/min.
    """

    def __init__(self, v_deadband: float = 0.06):
        self.v_deadband = v_deadband
        self.x = self.y = 0.0
        self.yaw: Optional[float] = None
        self._yaw0: Optional[float] = None
        self._t: Optional[float] = None

    @property
    def pose(self) -> Pose2:
        return self.x, self.y, self.yaw or 0.0

    def step(self, m: MotionStatus, t: float) -> Pose2:
        """Integrate one motion-status sample stamped t (seconds). The first sample defines odom's origin."""
        if self._yaw0 is None:
            self._yaw0 = m.yaw
        self.yaw = wrap(m.yaw - self._yaw0)
        if self._t is not None:
            dt = t - self._t
            if 0.0 < dt < 1.0:                       # a gap > 1 s is a dropout: do not extrapolate across it
                vx = 0.0 if abs(m.vx) < self.v_deadband else m.vx
                vy = 0.0 if abs(m.vy) < self.v_deadband else m.vy
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.x += (c * vx - s * vy) * dt
                self.y += (s * vx + c * vy) * dt
        self._t = t
        return self.pose


# --------------------------------------------------------------------------------------------- socket
@dataclass
class _Entry:
    t: float            # time.monotonic() when the frame was received
    items: dict


def _make_socket() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.5)
    return s


class Link:
    """One UDP socket, one receiver thread, one heartbeat thread; status pushes land in a timestamped cache.

    Exactly one Link per robot: a second socket from the same host displaced the first (E006 / lost pushes).
    `latest(code, max_age)` returns the newest Items for a status code, or None if older than max_age seconds:
    a stale status must never be mistaken for a live one (the heartbeat deadline is ~1 s; a missed one stops the
    pushes silently). `latest_entry` also gives the receive time, so a consumer can tell a NEW push from the cached
    one (the location report arrives ~2 Hz and is read at 10 Hz).

    A socket error (interface down, ICMP unreachable surfacing as OSError) reopens the socket and carries on;
    before 2026-11-09 both threads exited on the first one, silently, and the bridge was deaf until restarted.
    """

    def __init__(self, ip: str = ROBOT_IP, port: int = ROBOT_PORT, heartbeat_hz: float = 2.0, sock_factory=_make_socket):
        self.addr = (ip, port)
        self.heartbeat_period = 1.0 / heartbeat_hz
        self._make_socket = sock_factory
        self._sock: Optional[socket.socket] = None
        self._id = 0
        self._lock = threading.Lock()
        self._cache: Dict[Tuple[int, int], _Entry] = {}
        self._acks: Dict[Tuple[int, int], _Entry] = {}
        self._stop = threading.Event()
        self._threads = []
        self.rx_frames = 0
        self.rx_errors = 0
        self.reopens = 0

    # lifecycle
    def start(self) -> None:
        self._sock = self._make_socket()
        self._stop.clear()
        self._threads = [threading.Thread(target=self._rx_loop, name="m20_rx", daemon=True),
                         threading.Thread(target=self._hb_loop, name="m20_hb", daemon=True)]
        for th in self._threads:
            th.start()

    def stop(self) -> None:
        self._stop.set()
        for th in self._threads:
            th.join(timeout=2.0)
        if self._sock:
            self._sock.close()
            self._sock = None

    # sending
    def send(self, code: Tuple[int, int], items: Optional[dict] = None) -> int:
        with self._lock:
            self._id = (self._id + 1) & 0xFFFF
            msg_id = self._id
        self._sock.sendto(encode(msg_id, code, items), self.addr)
        return msg_id

    def send_msg(self, msg: Tuple[Tuple[int, int], dict]) -> int:
        return self.send(*msg)

    # receiving
    def latest_entry(self, code: Tuple[int, int], max_age: float = 2.0) -> Optional[_Entry]:
        with self._lock:
            e = self._cache.get(code)
        if e is None or time.monotonic() - e.t > max_age:
            return None
        return e

    def latest(self, code: Tuple[int, int], max_age: float = 2.0) -> Optional[dict]:
        e = self.latest_entry(code, max_age)
        return None if e is None else e.items

    def last_ack(self, code: Tuple[int, int], max_age: float = 2.0) -> Optional[dict]:
        with self._lock:
            e = self._acks.get(code)
        if e is None or time.monotonic() - e.t > max_age:
            return None
        return e.items

    def _reopen(self, where: str, err: OSError) -> None:
        with self._lock:
            self.reopens += 1
            n = self.reopens
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = self._make_socket()
        logging.getLogger("m20_udp").warning("m20 link %s: %s; socket reopened (#%d)", where, err, n)
        self._stop.wait(0.2)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(1 << 16)
            except socket.timeout:
                continue
            except OSError as e:
                if self._stop.is_set():
                    break
                self._reopen("rx", e)
                continue
            try:
                _, code, items = decode(data)
            except (ValueError, KeyError, json.JSONDecodeError):
                self.rx_errors += 1
                continue
            self.rx_frames += 1
            entry = _Entry(time.monotonic(), items)
            with self._lock:
                (self._acks if is_ack(items) else self._cache)[code] = entry

    def _hb_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.send(HEARTBEAT)
                for code in POLLS:                   # the 1 Hz pushes are re-polled with every heartbeat
                    self.send(code)
            except OSError as e:
                if self._stop.is_set():
                    break
                self._reopen("hb", e)
            self._stop.wait(self.heartbeat_period)
