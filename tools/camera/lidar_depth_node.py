#!/usr/bin/env python3
"""/camera/depth for the vendor fisheye from the front lidar: the no-new-hardware depth path (docs/REAL_ROBOT.md §2).

Subscribes the camera's /camera/image_raw + /camera/camera_info (for the stamp and intrinsics) and the lidar cloud
(LIDAR_TOPIC, default /rslidar_points_front, frame rslidar_front). For each camera frame it takes the newest cloud
within DEPTH_MAX_AGE (0.2 s), moves it into the camera optical frame through TF (lidar frame -> camera frame, both
static on the robot), projects it (tools/camera/lidar_depth.py) and publishes:
  /camera/depth   sensor_msgs/Image 32FC1 metres, 0 = no return, the SAME header stamp as the colour image
The keyframe recorder and the object-map builder pair colour and depth on identical stamps, so this node copies
the colour stamp rather than using the cloud's. The depth is sparse: every lidar point becomes a 5x5 pixel block.
"""
import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lidar_depth import depth_from_points, transform_points   # noqa: E402

LIDAR_TOPIC = os.environ.get("LIDAR_TOPIC", "/rslidar_points_front")
MAX_AGE = float(os.environ.get("DEPTH_MAX_AGE", "0.2"))
DILATE = int(os.environ.get("DEPTH_DILATE", "2"))


def tf_to_mat(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


class LidarDepth(Node):
    def __init__(self):
        super().__init__("m20_lidar_depth")
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.cloud = None            # (stamp seconds, frame, (N,3) float32)
        self.info = None
        self.T_cam_lidar = None
        self.pub = self.create_publisher(Image, "/camera/depth", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, LIDAR_TOPIC, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.n_pub = self.n_skip = 0
        self.get_logger().info(f"depth for /camera/image_raw from {LIDAR_TOPIC} (max age {MAX_AGE} s, dilate {DILATE})")

    def on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        with self.lock:
            self.cloud = (self.get_clock().now().nanoseconds * 1e-9, msg.header.frame_id, pts.astype(np.float32))

    def on_info(self, msg):
        self.info = msg

    def on_image(self, img):
        if self.info is None:
            return
        with self.lock:
            cloud = self.cloud
        now = self.get_clock().now().nanoseconds * 1e-9
        if cloud is None or now - cloud[0] > MAX_AGE:
            self.n_skip += 1
            if self.n_skip % 25 == 1:
                self.get_logger().warn(f"no lidar cloud within {MAX_AGE} s of the image ({self.n_skip} skipped)")
            return
        _, lidar_frame, pts = cloud
        if self.T_cam_lidar is None:
            try:
                self.T_cam_lidar = tf_to_mat(self.tf.lookup_transform(img.header.frame_id, lidar_frame, Time()))
                self.get_logger().info(f"TF {lidar_frame} -> {img.header.frame_id} found")
            except Exception as e:                # noqa: BLE001 (tf2 raises several exception types)
                self.get_logger().warn(f"no TF {lidar_frame} -> {img.header.frame_id} yet: {e}", throttle_duration_sec=5.0)
                return
        K = np.array(self.info.k).reshape(3, 3)
        depth = depth_from_points(transform_points(pts, self.T_cam_lidar), K, img.width, img.height, DILATE)
        out = Image()
        out.header = img.header                   # identical stamp: the recorder pairs colour and depth on it
        out.height, out.width, out.encoding, out.step = img.height, img.width, "32FC1", img.width * 4
        out.data = depth.tobytes()
        self.pub.publish(out)
        self.n_pub += 1
        if self.n_pub % 50 == 0:
            self.get_logger().info(f"{self.n_pub} depth images, last had {int((depth > 0).sum())} covered pixels")


def main():
    rclpy.init()
    rclpy.spin(LidarDepth())


if __name__ == "__main__":
    main()
