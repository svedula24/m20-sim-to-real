"""Project a lidar point cloud into a pinhole camera to make a sparse depth image (pure NumPy, unit-tested).

depth_from_points(points_cam, K, w, h, dilate) -> float32 (h, w) image in metres, 0 where no return:
points_cam are (N, 3) in the camera OPTICAL frame (x right, y down, z forward); a point lands on pixel
u = fx * x / z + cx, v = fy * y / z + cy; the nearest point wins a pixel; `dilate` grows each point into a
(2*dilate+1)^2 block so a 28k-point cloud covers an object's box well enough for the object-map builder's depth
read (it takes the median of the depths inside a box band, ignoring zeros).
"""
import numpy as np


def depth_from_points(points_cam: np.ndarray, K: np.ndarray, w: int, h: int, dilate: int = 2,
                      z_min: float = 0.3, z_max: float = 12.0) -> np.ndarray:
    depth = np.zeros((h, w), np.float32)
    if points_cam.size == 0:
        return depth
    p = np.asarray(points_cam, np.float64).reshape(-1, 3)
    z = p[:, 2]
    keep = (z > z_min) & (z < z_max) & np.isfinite(z)
    p = p[keep]
    if not len(p):
        return depth
    u = np.rint(K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]).astype(int)
    v = np.rint(K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]).astype(int)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z = u[inside], v[inside], p[inside, 2].astype(np.float32)
    order = np.argsort(-z)                     # far first, so the nearest point wins where several hit a pixel
    u, v, z = u[order], v[order], z[order]
    for du in range(-dilate, dilate + 1):
        for dv in range(-dilate, dilate + 1):
            uu, vv = u + du, v + dv
            ok = (uu >= 0) & (uu < w) & (vv >= 0) & (vv < h)
            depth[vv[ok], uu[ok]] = z[ok]
    return depth


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """(N, 3) points through a 4x4 homogeneous transform."""
    p = np.asarray(points, np.float64).reshape(-1, 3)
    return p @ T[:3, :3].T + T[:3, 3]
