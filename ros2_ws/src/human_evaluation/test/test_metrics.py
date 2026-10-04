import math

import numpy as np
import pytest

from human_evaluation.metrics import (
    compose,
    distance_stats,
    interpolate_pose,
    mean_and_std,
    path_length,
    robot_positions_in_map,
    yaw_from_quaternion,
)


def test_interpolation_between_samples():
    x, y, yaw = interpolate_pose([0.0, 2.0], [0.0, 4.0], [1.0, 3.0], [0.0, 1.0], [0.5, 1.0])
    assert x == pytest.approx([1.0, 2.0])
    assert y == pytest.approx([1.5, 2.0])
    assert yaw == pytest.approx([0.25, 0.5])


def test_interpolation_clamps_outside_the_series():
    x, _, _ = interpolate_pose([1.0, 2.0], [10.0, 20.0], [0.0, 0.0], [0.0, 0.0], [0.0, 5.0])
    assert x == pytest.approx([10.0, 20.0])


def test_yaw_interpolates_through_pi_the_short_way():
    _, _, yaw = interpolate_pose([0.0, 1.0], [0, 0], [0, 0], [3.0, -3.0], [0.5])
    assert math.cos(yaw[0]) == pytest.approx(-1.0, abs=1e-3)  # near pi, not near 0


def test_empty_series_is_rejected():
    with pytest.raises(ValueError):
        interpolate_pose([], [], [], [], [0.0])


def test_compose_rotates_then_translates():
    x, y = compose(1.0, 2.0, math.pi / 2, 3.0, 0.0)
    assert (x, y) == pytest.approx((1.0, 5.0))


def test_robot_position_in_map_combines_localization_and_odometry():
    odom = ([0.0, 10.0], [0.0, 10.0], [0.0, 0.0], [0.0, 0.0])  # 1 m/s along odom x
    map_to_odom = ([0.0, 10.0], [-4.0, -4.0], [0.0, 0.0], [0.0, 0.0])  # odom origin at (-4, 0)
    x, y = robot_positions_in_map(odom, map_to_odom, [2.5])
    assert (x[0], y[0]) == pytest.approx((-1.5, 0.0))


def test_robot_position_with_rotated_odom_frame():
    odom = ([0.0, 1.0], [2.0, 2.0], [0.0, 0.0], [0.0, 0.0])
    map_to_odom = ([0.0, 1.0], [1.0, 1.0], [1.0, 1.0], [math.pi / 2, math.pi / 2])
    x, y = robot_positions_in_map(odom, map_to_odom, [0.5])
    assert (x[0], y[0]) == pytest.approx((1.0, 3.0))


def test_distance_stats():
    stats = distance_stats(np.arange(1.0, 101.0))
    assert stats["min_distance"] == 1.0
    assert stats["p5_distance"] == pytest.approx(5.95)
    assert stats["mean_distance"] == pytest.approx(50.5)
    assert stats["samples"] == 100


def test_distance_stats_without_samples():
    assert distance_stats([]) == {
        "min_distance": None,
        "p5_distance": None,
        "mean_distance": None,
        "samples": 0,
    }


def test_path_length():
    assert path_length([0, 3, 3], [0, 0, 4]) == pytest.approx(7.0)
    assert path_length([1.0], [1.0]) == 0.0
    assert path_length([], []) == 0.0


def test_mean_and_std_use_the_sample_standard_deviation():
    mean, std = mean_and_std([1.0, 2.0, 3.0, 4.0])
    assert mean == pytest.approx(2.5)
    assert std == pytest.approx(1.2909944)
    assert mean_and_std([2.0]) == (2.0, None)
    assert mean_and_std([]) == (None, None)


def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0, 0, math.sin(0.4), math.cos(0.4)) == pytest.approx(0.8)
