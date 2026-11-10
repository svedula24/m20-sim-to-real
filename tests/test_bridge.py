import rclpy
from geometry_msgs.msg import Twist
from m20_msgs.msg import RobotMode
from m20_locomotion_bridge.bridge_node import LocomotionBridge


class SpySDK:
    def __init__(self): self.vel = []; self.stops = 0; self.regular = 0; self.alive_flag = True
    def connect(self): return True
    def send_velocity(self, vx, vy, wz): self.vel.append((vx, vy, wz))
    def request_gait(self, g): return True
    def stop(self): self.stops += 1; self.vel.append((0.0, 0.0, 0.0))   # the real stop() sends three zero frames
    def request_regular(self): self.regular += 1
    def alive(self): return self.alive_flag
    def status_line(self): return "spy"
    def healthy(self): return True


def arm(bridge):
    m = RobotMode(); m.armed = True; m.mode = RobotMode.MODE_TELEOP
    rclpy.deliver("/m20/mode", m)

def test_ignores_cmd_when_disarmed():
    b = LocomotionBridge(); b.sdk = SpySDK()
    rclpy.deliver("/cmd_vel", Twist())
    assert b.sdk.vel == []

def test_forwards_cmd_when_armed():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    t = Twist(); t.linear.x = 1.0
    rclpy.deliver("/cmd_vel", t)
    assert b.sdk.vel[-1][0] == 1.0

def test_watchdog_halts_on_timeout():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    rclpy.set_time(10.0)
    rclpy.deliver("/cmd_vel", Twist())
    rclpy.set_time(11.1)          # > 1.0 s default timeout
    b.watchdog()
    assert b.stopped and b.sdk.stops >= 1
    assert b.sdk.vel[-1] == (0.0, 0.0, 0.0)

def test_watchdog_quiet_within_timeout():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    rclpy.set_time(10.0)
    rclpy.deliver("/cmd_vel", Twist())
    rclpy.set_time(10.3)
    b.watchdog()
    assert not b.stopped

def test_disarm_halts_immediately():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    rclpy.deliver("/cmd_vel", Twist())
    m = RobotMode(); m.armed = False
    rclpy.deliver("/m20/mode", m)
    assert b.stopped and b.sdk.stops >= 1

def test_halt_is_one_stop_and_no_velocity_after_it():
    # the old halt() called send_velocity(0,0,0) after stop(), which turned the 20 Hz stream back on: zero frames
    # forever, pendant locked out. Now: stop() once, then nothing, however often the watchdog runs.
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b)
    rclpy.set_time(10.0); rclpy.deliver("/cmd_vel", Twist())
    rclpy.set_time(11.2); b.watchdog()
    n_stop, n_vel = b.sdk.stops, len(b.sdk.vel)
    rclpy.set_time(12.4); b.watchdog(); b.watchdog()
    assert (b.sdk.stops, len(b.sdk.vel)) == (n_stop, n_vel) == (1, 2)   # one cmd, one stop's zeros, no more

def test_arming_asks_for_regular_mode_once():
    b = LocomotionBridge(); b.sdk = SpySDK()
    arm(b); arm(b)                                     # the latched mode is re-delivered on every change
    assert b.sdk.regular == 1
    m = RobotMode(); m.armed = False; rclpy.deliver("/m20/mode", m)
    arm(b)
    assert b.sdk.regular == 2                          # a new arm asks again

def test_connection_follows_the_robots_status():
    # connect() failing at start used to leave `connected` False forever (a robot powered on after the bridge was
    # never accepted); a robot that stops answering used to stay "connected" with its last status
    b = LocomotionBridge(); b.sdk = SpySDK()
    b.connected = False
    b.heartbeat()
    assert b.connected
    arm(b); rclpy.deliver("/cmd_vel", Twist())
    b.sdk.alive_flag = False
    b.heartbeat()
    assert not b.connected and b.stopped and b.sdk.stops == 1
