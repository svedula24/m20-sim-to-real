import os
import sys

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "tools", "camera"))
from lidar_depth import depth_from_points, transform_points  # noqa: E402

K = np.array([[320.0, 0.0, 320.0], [0.0, 320.0, 240.0], [0.0, 0.0, 1.0]])   # 640x480, 90 deg hfov


def test_point_on_the_axis_lands_at_the_principal_point():
    d = depth_from_points(np.array([[0.0, 0.0, 2.5]]), K, 640, 480, dilate=0)
    assert d[240, 320] == 2.5 and (d > 0).sum() == 1


def test_dilation_covers_a_block_and_nearest_point_wins():
    pts = np.array([[0.0, 0.0, 4.0], [0.0, 0.0, 1.5]])
    d = depth_from_points(pts, K, 640, 480, dilate=2)
    assert d[240, 320] == 1.5 and (d > 0).sum() == 25
    assert d[238, 318] == 1.5 and d[243, 323] == 0.0


def test_points_behind_or_outside_are_dropped():
    pts = np.array([[0.0, 0.0, -1.0], [10.0, 0.0, 1.0], [0.0, 0.0, 0.1], [0.0, 0.0, 50.0]])
    d = depth_from_points(pts, K, 640, 480, dilate=0)
    assert (d > 0).sum() == 0
    assert depth_from_points(np.zeros((0, 3)), K, 640, 480).shape == (480, 640)


def test_projection_geometry_right_and_down():
    # 1 m right and 0.5 m down of the axis at 2 m: u = 320 * 0.5 + 320 = 480, v = 320 * 0.25 + 240 = 320
    d = depth_from_points(np.array([[1.0, 0.5, 2.0]]), K, 640, 480, dilate=0)
    assert d[320, 480] == 2.0


def test_transform_points_applies_rotation_and_translation():
    T = np.eye(4)
    T[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]          # +90 deg about z
    T[:3, 3] = [1.0, 0.0, 0.0]
    out = transform_points(np.array([[1.0, 0.0, 0.0]]), T)
    assert np.allclose(out, [[1.0, 1.0, 0.0]])
