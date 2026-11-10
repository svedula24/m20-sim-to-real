"""m20_locomotion_bridge: the ONLY node that talks to the Deep Robotics M20. Everything above it is vendor-agnostic.

Contract (unchanged from the simulator phase, so Nav2, the commander, the mission server and NL navigation run
as they did in simulation):
  subscribes  /cmd_vel            (geometry_msgs/Twist)    body velocity, m/s and rad/s, REP-103 signs
  subscribes  /m20/gait_request   (m20_msgs/GaitRequest)   stand / sit / "wheeled" = the velocity controller
  subscribes  /m20/mode           (m20_msgs/RobotMode)     commands are forwarded only while armed
  publishes   /odom + TF odom->base_link                   dead-reckoned from the robot's reported body velocity
  publishes   /m20/vendor_pose    (geometry_msgs/PoseStamped, map)  the robot's own localization, ONE message per
                                                           new report, stamped at the report's receive time
                                                           (minus vendor_pose_delay_s). map->odom is published from
                                                           it by tools/nav/map_tf_adapter.py, the same file the sim
                                                           rehearsed with; the bridge publishes no map->odom since
                                                           2026-11-09 (it recomputed the correction 10 x/s from a
                                                           report that changed 2 x/s: the map pose stood still, then
                                                           jumped 0.15 m per report at 0.3 m/s)
  publishes   /IMU_DATA           (drdds/ImuData)          roll/pitch/omega_z for the commander's tip-over watch
  publishes   /m20/health         (m20_msgs/HealthReport)  1 Hz; WARN while the robot is not in the velocity controller
  action      /navigate_to_pose   (nav2_msgs/NavigateToPose) OPTIONAL (vendor_nav:=true): the robot's own
                                                           navigation stands in for Nav2 (docs/REAL_ROBOT.md stage A)
Watchdog: no /cmd_vel for `cmd_timeout_s` (1.0 s) -> zero velocity. The robot itself also brakes 0.5 s after the
last axis frame, so a dead bridge stops the robot too.

Backends (`sdk_backend` parameter): "stub" (default: no robot, used by the unit tests) or "m20_udp" (the real
robot over the inspection protocol, see m20_udp.py). Every vendor call stays inside the backend class.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from m20_msgs.msg import GaitRequest, RobotMode, HealthReport

from . import m20_udp as proto


class VendorSDK:
    """No-robot backend: accepts everything, moves nothing. Keeps the unit tests ROS- and robot-free."""

    def connect(self) -> bool:
        return True

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        pass

    def request_gait(self, gait: int) -> bool:
        return True

    def stop(self) -> None:
        pass

    def request_regular(self) -> None:
        pass

    def alive(self) -> bool:
        return True

    def status_line(self) -> str:
        return "stub backend"

    def healthy(self) -> bool:
        return True


class M20UdpSDK:
    """The real M20 over UDP (m20_udp.Link). Owns the one socket; streams axis commands at 20 Hz; turns the
    robot's status pushes into /odom, TF and /IMU_DATA.

    Parameters (all on the bridge node): robot_ip, robot_port, vx_max, vy_max, wz_max, vx_dead, vy_dead, wz_dead
    (m20_udp.SpeedTable: MEASURE THEM on your robot, tools/robot/m20_probe.py drive), stream_hz (20),
    v_deadband (0.06 m/s: reported LinearX reads ~0.05 at rest), vendor_pose_delay_s (0.0: how far behind the
    motion the robot's position report is; the sim assumed 0.1, measure it on the robot at walk step 3).

    Axis frames go out only while the robot's usage mode is Regular (BasicStatus.ControlUsageMode 0): Navigation
    mode ignores them, and the robot drops back to Regular on its own ~3 s after entering its walking controller,
    so arming asks for Regular once and every frame is gated on the live status.
    """

    def __init__(self, node: Node):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from tf2_ros import TransformBroadcaster
        self.node = node
        self._Odometry, self._PoseStamped, self._TransformStamped = Odometry, PoseStamped, TransformStamped
        g = lambda n, d: (node.declare_parameter(n, d), node.get_parameter(n).value)[1]
        self.link = proto.Link(g("robot_ip", proto.ROBOT_IP), int(g("robot_port", proto.ROBOT_PORT)))
        self.table = proto.SpeedTable(g("vx_max", 2.1), g("vy_max", 0.62), g("wz_max", 1.8),
                                      g("vx_dead", 0.0), g("vy_dead", 0.16), g("wz_dead", 0.15))
        self.reckoner = proto.DeadReckoner(g("v_deadband", 0.06))
        self.delay = float(g("vendor_pose_delay_s", 0.0))
        stream_hz = float(g("stream_hz", 20.0))
        self._target = (0.0, 0.0, 0.0)
        self._streaming = False               # True between the first send_velocity and stop()
        self._no_motion_warned = False        # the no-MotionStatus fallback is logged once
        self._last_loc_t = None               # receive time of the last location report published
        self._lock = threading.Lock()
        self.navigating = False               # vendor navigation owns the robot: axis commands are refused
        self.basic = None                     # latest proto.BasicStatus
        self.location = None                  # latest proto.LocationStatus
        self.battery = None
        self.odom_pub = node.create_publisher(Odometry, "/odom", 10)
        self.pose_pub = node.create_publisher(PoseStamped, "/m20/vendor_pose", 10)
        self.tf = TransformBroadcaster(node)
        self._imu_pub = None
        try:
            from drdds.msg import ImuData
            self._ImuData = ImuData
            self._imu_pub = node.create_publisher(ImuData, "/IMU_DATA", 10)
        except ImportError:
            node.get_logger().warn("drdds msgs unavailable: /IMU_DATA not published, the commander's tip-over watch is off")
        node.create_timer(1.0 / stream_hz, self._stream)
        node.create_timer(0.1, self._state)

    # ---- the backend contract
    def connect(self) -> bool:
        self.link.start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            if self.link.latest(proto.STATUS_BASIC, 3.0) is not None:
                self.node.get_logger().info(f"M20 answering at {self.link.addr[0]}:{self.link.addr[1]}: {self.status_line()}")
                return True
            time.sleep(0.1)
        self.node.get_logger().error(f"no status from the M20 at {self.link.addr[0]}:{self.link.addr[1]} in 5 s "
                                     "(wire? robot on? another UDP client holding the pushes?)")
        return False

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        if self.navigating:
            return                           # Navigation usage mode ignores axis commands anyway; do not fight it
        with self._lock:
            self._target = (vx, vy, wz)
            self._streaming = True

    def stop(self) -> None:
        """Three zero frames, then silence: the robot brakes within 0.5 s of the last frame, and silence hands the
        robot back to the pendant (a client that keeps sending zeros stays its one UDP client)."""
        with self._lock:
            self._target = (0.0, 0.0, 0.0)
            self._streaming = False
        for _ in range(3):
            self._send_axis(0.0, 0.0, 0.0)

    def request_regular(self) -> None:
        self.link.send_msg(proto.usage_mode(proto.MODE_REGULAR))

    def alive(self) -> bool:
        return self.link.latest(proto.STATUS_BASIC, 3.0) is not None

    def request_gait(self, g: int) -> bool:
        """GaitRequest -> the robot's motion state. GAIT_WHEELED = the velocity controller (RL Control, 17)."""
        if g == GaitRequest.GAIT_STAND:
            self.link.send_msg(proto.motion_state(proto.MS_STAND))
        elif g == GaitRequest.GAIT_WHEELED:
            self.link.send_msg(proto.gait(proto.GAIT_FLAT_AGILE))
            time.sleep(1.5)                  # the robot needs a moment between gait and controller changes
            self.link.send_msg(proto.motion_state(proto.MS_RL_CONTROL))
        elif g == GaitRequest.GAIT_SIT:
            self.link.send_msg(proto.motion_state(proto.MS_SIT))
        else:
            self.node.get_logger().warn(f"gait {g} has no M20 equivalent (legged/hybrid): ignored")
            return False
        return True

    def soft_estop(self) -> None:
        self.link.send_msg(proto.motion_state(proto.MS_SOFT_ESTOP))

    def healthy(self) -> bool:
        b = self.basic
        return b is not None and b.motion_state == proto.MS_RL_CONTROL and not b.hard_estop

    def status_line(self) -> str:
        b, loc = self.basic, self.location
        parts = []
        if b:
            parts.append(f"state={b.motion_state} gait=0x{b.gait:04x} mode={b.usage_mode} HES={int(b.hard_estop)}")
        if loc:
            parts.append(f"loc={'ok' if loc.ok else 'LOST'} ({loc.x:+.2f},{loc.y:+.2f},{loc.yaw:+.2f})")
        if self.battery is not None:
            parts.append(f"battery={self.battery:.0f}%")
        return " ".join(parts) or "no status"

    # ---- timers
    def _send_axis(self, vx, vy, wz):
        try:
            self.link.send_msg(proto.axis(*self.table.ratios(vx, vy, wz)))
        except OSError as e:
            self.node.get_logger().error(f"axis send failed: {e}")

    def _stream(self):
        with self._lock:
            streaming, target = self._streaming, self._target
        if not streaming:
            return
        if self.basic is None or self.basic.usage_mode != proto.MODE_REGULAR:
            self.node.get_logger().warn(f"axis frame withheld: usage mode is {None if self.basic is None else self.basic.usage_mode}, "
                                        "not Regular (0); Navigation mode ignores axis commands", throttle_duration_sec=5.0)
            return
        self._send_axis(*target)

    def _state(self):
        now = self.node.get_clock().now()
        b = self.link.latest(proto.STATUS_BASIC, 3.0)
        if b is not None:
            self.basic = proto.parse_basic(b)
        d = self.link.latest(proto.STATUS_DEVICE, 5.0)
        if d is not None:
            self.battery = proto.parse_battery(d)
        e = self.link.latest_entry(proto.STATUS_NAV, 3.0)
        if e is not None:
            self.location = proto.parse_location(e.items)
            if e.t != self._last_loc_t:          # a NEW report (the cache is read 10 x/s, the robot sends ~2 x/s)
                self._last_loc_t = e.t
                if self.location.ok:
                    self._publish_vendor_pose(now, e.t)
        m = self.link.latest(proto.STATUS_MOTION, 1.0)
        if m is None:
            # No MotionStatus (1002/4): the robot always sends it; BrightAI's protocol emulator does not model it
            # (first run against it, 2026-11-05: nothing was published and the stack looked dead). Then the vendor
            # pose IS the odometry (odom == map, so the adapter's map->odom comes out as the identity) and the twist
            # is what we are commanding. Unchanged path on the robot.
            if self.location is None or not self.location.ok:
                return
            if not self._no_motion_warned:
                self.node.get_logger().warn("no MotionStatus (1002/4) from this sender: /odom follows the vendor pose "
                                            "(an emulator, not the robot?)")
                self._no_motion_warned = True
            with self._lock:
                vx, vy, wz = self._target if self._streaming else (0.0, 0.0, 0.0)
            ms = proto.MotionStatus(vx, vy, wz, 0.0, 0.0, self.location.yaw, 0.0)
            self._publish_odom(now, (self.location.x, self.location.y, self.location.yaw), ms)
            return
        ms = proto.parse_motion(m)
        odom_base = self.reckoner.step(ms, time.monotonic())
        self._publish_odom(now, odom_base, ms)
        if self._imu_pub is not None:
            imu = self._ImuData()
            imu.header.stamp = now.to_msg()
            imu.data.roll, imu.data.pitch, imu.data.yaw = ms.roll, ms.pitch, ms.yaw
            imu.data.omega_z = ms.wz
            self._imu_pub.publish(imu)

    def _publish_vendor_pose(self, now, rx_monotonic):
        """Stamped when the report was RECEIVED (ROS time now minus how long ago that was), minus the configured
        report delay: the adapter looks up odometry at this stamp, so the stamp must be the time the pose was true."""
        ps = self._PoseStamped()
        stamp_ns = now.nanoseconds - int((time.monotonic() - rx_monotonic + self.delay) * 1e9)
        ps.header.stamp.sec, ps.header.stamp.nanosec = stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y = self.location.x, self.location.y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(self.location.yaw / 2), math.cos(self.location.yaw / 2)
        self.pose_pub.publish(ps)

    def _tf(self, now, parent, child, pose):
        t = self._TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = pose[0], pose[1]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        return t

    def _publish_odom(self, now, pose, ms):
        o = self._Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id, o.child_frame_id = "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = pose[0], pose[1]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        o.twist.twist.linear.x = 0.0 if abs(ms.vx) < self.reckoner.v_deadband else ms.vx
        o.twist.twist.linear.y = 0.0 if abs(ms.vy) < self.reckoner.v_deadband else ms.vy
        o.twist.twist.angular.z = ms.wz
        self.odom_pub.publish(o)
        self.tf.sendTransform(self._tf(now, "odom", "base_link", pose))


class VendorNav:
    """nav2_msgs/NavigateToPose served by the robot's own navigation (1003/1), so the mission server and NL
    navigation run unchanged without Nav2 (docs/REAL_ROBOT.md, stage A).

    Goal poses are in the vendor map frame (the bridge's `map`). The robot is switched to Navigation usage
    mode for the task and back to Regular afterwards; axis commands are refused meanwhile. A task "ending" is not
    arrival (legs have ended 5-9 m short): success is the vendor pose within `tolerance` (0.35 m) of the goal.
    Failure: nav ErrorCode set, localization lost, no progress of 0.12 m in 20 s (the vendor's Auto mode can hold
    position forever), or `timeout_s`. Parameters: nav_mode (0 Straight: accurate direct leg; 1 Auto: vendor
    planner), tolerance, timeout_s.
    """

    def __init__(self, node: Node, sdk: M20UdpSDK):
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from nav2_msgs.action import NavigateToPose
        self.node, self.sdk = node, sdk
        g = lambda n, d: (node.declare_parameter(n, d), node.get_parameter(n).value)[1]
        self.nav_mode, self.tolerance, self.timeout_s = int(g("nav_mode", 0)), float(g("tolerance", 0.35)), float(g("timeout_s", 180.0))
        self._cancel = CancelResponse
        self._Goal = GoalResponse
        self.server = ActionServer(node, NavigateToPose, "/navigate_to_pose", execute_callback=self.execute,
                                   goal_callback=lambda req: GoalResponse.ACCEPT,
                                   cancel_callback=lambda gh: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self._NavigateToPose = NavigateToPose
        node.get_logger().info(f"vendor navigation serving /navigate_to_pose (nav_mode={self.nav_mode}, tol={self.tolerance} m)")

    def _wait_mode(self, mode, secs=4.0) -> bool:
        self.sdk.link.send_msg(proto.usage_mode(mode))
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            time.sleep(0.25)
            if self.sdk.basic is not None and self.sdk.basic.usage_mode == mode:
                return True
        return False

    def execute(self, gh):
        res = self._NavigateToPose.Result()
        p = gh.request.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        goal = (p.position.x, p.position.y)
        log = self.node.get_logger()
        if self.sdk.location is None or not self.sdk.location.ok:
            log.error("vendor nav: localization not ok, goal refused")
            gh.abort()
            return res
        self.sdk.navigating = True
        self.sdk.stop()
        try:
            if not self._wait_mode(proto.MODE_NAVIGATION):
                log.error("vendor nav: robot did not enter Navigation usage mode")
                gh.abort()
                return res
            self.sdk.link.send_msg(proto.nav_task(goal[0], goal[1], yaw, nav_mode=self.nav_mode))
            log.info(f"vendor nav: task to ({goal[0]:.2f}, {goal[1]:.2f}, {yaw:.2f})")
            t0 = time.monotonic()
            best, best_t = None, t0
            while True:
                time.sleep(0.5)
                loc = self.sdk.location
                now = time.monotonic()
                if gh.is_cancel_requested:
                    self.sdk.link.send_msg(proto.nav_cancel())
                    gh.canceled()
                    log.warn("vendor nav: canceled")
                    return res
                if loc is None or not loc.ok:
                    self.sdk.link.send_msg(proto.nav_cancel())
                    log.error("vendor nav: localization lost during the task")
                    gh.abort()
                    return res
                dist = math.hypot(loc.x - goal[0], loc.y - goal[1])
                fb = self._NavigateToPose.Feedback()
                fb.distance_remaining = dist
                gh.publish_feedback(fb)
                if dist <= self.tolerance:
                    log.info(f"vendor nav: arrived, {dist:.2f} m from the goal after {now - t0:.0f} s")
                    gh.succeed()
                    return res
                if best is None or dist < best - 0.12:
                    best, best_t = dist, now
                elif now - best_t > 20.0:
                    self.sdk.link.send_msg(proto.nav_cancel())
                    log.error(f"vendor nav: no progress for 20 s at {dist:.2f} m from the goal")
                    gh.abort()
                    return res
                if loc.nav_status == 0xFF or (loc.nav_error not in (0, 0xE01F) and now - t0 > 3.0):
                    log.error(f"vendor nav: robot reports failure status={loc.nav_status} error=0x{loc.nav_error:x}")
                    gh.abort()
                    return res
                if now - t0 > self.timeout_s:
                    self.sdk.link.send_msg(proto.nav_cancel())
                    log.error(f"vendor nav: timeout after {self.timeout_s:.0f} s, {dist:.2f} m from the goal")
                    gh.abort()
                    return res
        finally:
            self._wait_mode(proto.MODE_REGULAR)
            self.sdk.navigating = False


class LocomotionBridge(Node):
    def __init__(self):
        super().__init__("m20_locomotion_bridge")
        # 1.0 s: Nav2's BT has natural /cmd_vel gaps of 0.5-0.8 s between FollowPath attempts and recovery
        # transitions; at 0.5 s the watchdog halted on every transition. Still a real failsafe: a dead Nav2 halts
        # the robot in 1 s (and the robot itself brakes 0.5 s after the last axis frame).
        self.declare_parameter("cmd_timeout_s", 1.0)
        self.timeout = self.get_parameter("cmd_timeout_s").value
        self.declare_parameter("sdk_backend", "stub")
        self.declare_parameter("vendor_nav", False)
        backend = self.get_parameter("sdk_backend").value

        self.sdk = M20UdpSDK(self) if backend == "m20_udp" else VendorSDK()
        self.connected = self.sdk.connect()
        self.armed = False
        self.last_cmd_time = self.get_clock().now()
        self.stopped = True
        self.vendor_nav = None
        if backend == "m20_udp" and self.get_parameter("vendor_nav").value:
            self.vendor_nav = VendorNav(self, self.sdk)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(GaitRequest, "/m20/gait_request", self.on_gait, 10)
        # /m20/mode is latched by the Commander (TRANSIENT_LOCAL); match it so a (re)started bridge receives the
        # current mode immediately and arms.
        mode_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotMode, "/m20/mode", self.on_mode, mode_qos)
        self.health_pub = self.create_publisher(HealthReport, "/m20/health", 10)

        self.create_timer(0.1, self.watchdog)   # 10 Hz
        self.create_timer(1.0, self.heartbeat)  # 1 Hz
        self.get_logger().info(f"Bridge up. backend={backend} connected={self.connected}")

    def on_mode(self, msg: RobotMode):
        was_armed, self.armed = self.armed, msg.armed
        if self.armed and not was_armed:
            self.sdk.request_regular()        # axis frames need Regular usage mode; _stream checks the live status
        if not self.armed:
            self.halt("disarmed by commander")

    def on_cmd_vel(self, msg: Twist):
        if not (self.armed and self.connected):
            return
        self.last_cmd_time = self.get_clock().now()
        self.stopped = False
        self.sdk.send_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def on_gait(self, msg: GaitRequest):
        if not (self.armed and self.connected):
            self.get_logger().warn(f"gait request from {msg.requester} ignored (disarmed)")
            return
        ok = self.sdk.request_gait(msg.gait)
        self.get_logger().info(f"gait {msg.gait} by {msg.requester}: {'ok' if ok else 'REJECTED'}")

    def watchdog(self):
        if self.stopped:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.halt(f"cmd_vel timeout ({age:.2f}s > {self.timeout}s)")

    def halt(self, reason: str):
        """stop() sends three zero frames and ends the stream. Nothing else: the old extra send_velocity(0, 0, 0)
        turned streaming back on, so zero frames went out at 20 Hz forever and the pendant stayed locked out."""
        if not self.stopped:
            self.get_logger().warn(f"HALT: {reason}")
        self.sdk.stop()
        self.stopped = True

    def heartbeat(self):
        alive = self.sdk.alive()               # a basic status younger than 3 s; a robot powered on late is accepted
        if alive and not self.connected:
            self.get_logger().info(f"robot answering: {self.sdk.status_line()}")
        elif not alive and self.connected:
            self.halt("robot status stale (no BasicStatus for 3 s)")
        self.connected = alive
        h = HealthReport()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        if not self.connected:
            h.status, h.message = HealthReport.STATUS_ERROR, "robot not connected"
        elif not self.sdk.healthy():
            h.status, h.message = HealthReport.STATUS_WARN, self.sdk.status_line()
        else:
            h.status, h.message = HealthReport.STATUS_OK, self.sdk.status_line()
        self.health_pub.publish(h)


def main():
    rclpy.init()
    node = LocomotionBridge()
    if node.vendor_nav is not None:
        from rclpy.executors import MultiThreadedExecutor
        ex = MultiThreadedExecutor(num_threads=4)      # the action's execute callback blocks; timers keep running
        ex.add_node(node)
        ex.spin()
    else:
        rclpy.spin(node)


if __name__ == "__main__":
    main()
