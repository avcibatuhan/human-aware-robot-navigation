import pytest

from human_tracking.projection import Intrinsics, project_pixel

K = Intrinsics(fx=500.0, fy=400.0, cx=320.0, cy=240.0)


def test_principal_point_projects_onto_optical_axis():
    assert project_pixel(320.0, 240.0, 3.0, K) == (0.0, 0.0, 3.0)


def test_offsets_scale_with_depth_and_focal_length():
    x, y, z = project_pixel(420.0, 200.0, 2.0, K)
    assert x == pytest.approx((420 - 320) * 2.0 / 500)  # 0.4 m right
    assert y == pytest.approx((200 - 240) * 2.0 / 400)  # 0.2 m up (negative Y)
    assert z == 2.0


def test_round_trip_with_forward_projection():
    point = (0.7, -0.3, 4.2)
    u = K.fx * point[0] / point[2] + K.cx
    v = K.fy * point[1] / point[2] + K.cy
    assert project_pixel(u, v, point[2], K) == pytest.approx(point)


def test_intrinsics_from_camera_info_k():
    k = [467.74, 0.0, 320.0, 0.0, 468.1, 240.0, 0.0, 0.0, 1.0]
    assert Intrinsics.from_k(k) == Intrinsics(fx=467.74, fy=468.1, cx=320.0, cy=240.0)


def test_zero_focal_length_is_rejected():
    with pytest.raises(ValueError):
        project_pixel(1.0, 1.0, 1.0, Intrinsics(0.0, 400.0, 320.0, 240.0))
