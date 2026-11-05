#!/usr/bin/env python3
"""The M20's vendor fisheye camera as a pinhole ROS camera: RTSP in, rectified rgb8 + CameraInfo out.

The robot streams H.265 from the motion host (rtsp://10.21.31.103:8554/video1 and /video2, 800x600). The object
map builder, the keyframe recorder and the NL navigation detector all assume a pinhole camera (fx, fy, cx, cy,
no distortion), so this node undistorts each frame with the calibrated Kannala-Brandt model (cv2.fisheye) onto a
pinhole of the requested field of view and publishes:
  /camera/image_raw     sensor_msgs/Image rgb8, frame camera_optical_link
  /camera/camera_info   sensor_msgs/CameraInfo (plumb_bob, zero distortion), same stamp
Pair it with tools/camera/lidar_depth_node.py for /camera/depth, or use an RGB-D camera instead (docs/REAL_ROBOT.md §2).

Environment: CAM_URL (default the front stream), CAM_HZ (5), CAM_FRAME (camera_optical_link), CAM_FOV_DEG (90,
the pinhole's horizontal field of view; the fisheye sees 113), CAM_OUT_W/H (640x480). Intrinsics: CAM_FX, CAM_FY,
CAM_CX, CAM_CY, CAM_D (four comma-separated Kannala-Brandt coefficients) default to the front camera's calibration
at 800x600; recalibrate if the stream resolution differs (1280x720 has been seen after a video power cycle).
Stamps are the host clock at capture: the robot's clocks run tens of seconds off.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

URL = os.environ.get("CAM_URL", "rtsp://10.21.31.103:8554/video1")
HZ = float(os.environ.get("CAM_HZ", "5"))
FRAME = os.environ.get("CAM_FRAME", "camera_optical_link")
FOV_DEG = float(os.environ.get("CAM_FOV_DEG", "90"))
OUT_W, OUT_H = int(os.environ.get("CAM_OUT_W", "640")), int(os.environ.get("CAM_OUT_H", "480"))
K_IN = np.array([[float(os.environ.get("CAM_FX", "396.01")), 0.0, float(os.environ.get("CAM_CX", "386.38"))],
                 [0.0, float(os.environ.get("CAM_FY", "397.14")), float(os.environ.get("CAM_CY", "285.00"))],
                 [0.0, 0.0, 1.0]])
D_IN = np.array([float(v) for v in os.environ.get("CAM_D", "0.04429,0.002556,-0.03711,0.016065").split(",")]).reshape(4, 1)


def pinhole_K(w: int, h: int, hfov_deg: float) -> np.ndarray:
    """Output intrinsics: fx from the horizontal field of view, square pixels, principal point at the centre."""
    fx = (w / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    return np.array([[fx, 0.0, w / 2.0], [0.0, fx, h / 2.0], [0.0, 0.0, 1.0]])


class FisheyeCamera(Node):
    def __init__(self):
        super().__init__("m20_fisheye_camera")
        self.K_out = pinhole_K(OUT_W, OUT_H, FOV_DEG)
        self.map1 = self.map2 = None
        self.in_size = None
        self.img_pub = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.cap = cv2.VideoCapture(URL, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f"cannot open {URL} (robot on? MediaMTX/RTSP reachable on the wire?)")
            sys.exit(1)
        self.n = 0
        self.t_last = 0.0
        self.get_logger().info(f"streaming {URL} -> /camera/image_raw {OUT_W}x{OUT_H} at {HZ} Hz, hfov {FOV_DEG} deg")
        self.create_timer(0.005, self.grab)

    def _maps(self, w, h):
        if self.in_size != (w, h):
            if (w, h) != (800, 600):
                self.get_logger().warn(f"stream is {w}x{h}, the intrinsics are for 800x600: recalibrate or scale CAM_FX/FY/CX/CY")
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K_IN, D_IN, np.eye(3), self.K_out, (OUT_W, OUT_H), cv2.CV_16SC2)
            self.in_size = (w, h)

    def grab(self):
        ok = self.cap.grab()                      # drain the stream; decode only at the publish rate
        if not ok:
            self.get_logger().error("RTSP stream ended", throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if now - self.t_last < 1.0 / HZ:
            return
        ok, bgr = self.cap.retrieve()
        if not ok:
            return
        self.t_last = now
        h, w = bgr.shape[:2]
        self._maps(w, h)
        rect = cv2.remap(bgr, self.map1, self.map2, cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(rect, cv2.COLOR_BGR2RGB)
        stamp = self.get_clock().now().to_msg()
        img = Image()
        img.header.stamp, img.header.frame_id = stamp, FRAME
        img.height, img.width, img.encoding, img.step = OUT_H, OUT_W, "rgb8", OUT_W * 3
        img.data = rgb.tobytes()
        info = CameraInfo()
        info.header = img.header
        info.height, info.width = OUT_H, OUT_W
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(v) for v in self.K_out.flatten()]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.K_out[0, 0], 0.0, self.K_out[0, 2], 0.0, 0.0, self.K_out[1, 1], self.K_out[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.img_pub.publish(img)
        self.info_pub.publish(info)
        self.n += 1
        if self.n % 50 == 0:
            self.get_logger().info(f"{self.n} frames")


def main():
    rclpy.init()
    rclpy.spin(FisheyeCamera())


if __name__ == "__main__":
    main()
