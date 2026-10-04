import math

import pytest

from human_tracking.transform import apply_transform, rotate

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def yaw(angle):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def test_identity():
    assert apply_transform((0.0, 0.0, 0.0), IDENTITY, (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_translation_only():
    assert apply_transform((1.0, -2.0, 0.5), IDENTITY, (1.0, 2.0, 3.0)) == (2.0, 0.0, 3.5)


def test_yaw_90_degrees():
    assert rotate(yaw(math.pi / 2), (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))


def test_rotation_is_applied_before_translation():
    point = apply_transform((10.0, 0.0, 0.0), yaw(math.pi), (1.0, 0.0, 0.0))
    assert point == pytest.approx((9.0, 0.0, 0.0))


def test_optical_frame_to_robot_frame():
    # ROS optical frame (Z forward, X right, Y down) -> body frame (X forward, Y left, Z up)
    optical_to_body = (-0.5, 0.5, -0.5, 0.5)
    assert rotate(optical_to_body, (0.0, 0.0, 3.0)) == pytest.approx((3.0, 0.0, 0.0))
    assert rotate(optical_to_body, (1.0, 0.0, 0.0)) == pytest.approx((0.0, -1.0, 0.0))
    assert rotate(optical_to_body, (0.0, 1.0, 0.0)) == pytest.approx((0.0, 0.0, -1.0))
