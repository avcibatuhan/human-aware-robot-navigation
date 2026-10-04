"""Navigation metrics computed from plain arrays. No ROS imports."""

import math

import numpy as np


def interpolate_pose(times, xs, ys, yaws, query_times):
    """Linearly interpolate a 2D pose series at ``query_times``.

    Yaw is unwrapped before interpolating so that a turn through +-pi does not
    sweep the long way round. Queries outside the series are clamped to its
    first or last pose.
    """
    times = np.asarray(times, dtype=float)
    if times.size == 0:
        raise ValueError("pose series is empty")
    query = np.asarray(query_times, dtype=float)
    x = np.interp(query, times, np.asarray(xs, dtype=float))
    y = np.interp(query, times, np.asarray(ys, dtype=float))
    yaw = np.interp(query, times, np.unwrap(np.asarray(yaws, dtype=float)))
    return x, y, yaw


def compose(parent_x, parent_y, parent_yaw, child_x, child_y):
    """Express a point given in a child frame in the parent's frame.

    ``parent_*`` is the pose of the child frame in the parent frame.
    """
    cos, sin = np.cos(parent_yaw), np.sin(parent_yaw)
    return parent_x + cos * child_x - sin * child_y, parent_y + sin * child_x + cos * child_y


def robot_positions_in_map(odom, map_to_odom, query_times):
    """Robot x, y in the map frame at ``query_times``.

    ``odom`` is the robot pose in the odom frame and ``map_to_odom`` the pose of
    the odom frame in the map frame, each as ``(times, xs, ys, yaws)``. Both are
    interpolated in time and then composed.
    """
    robot_x, robot_y, _ = interpolate_pose(*odom, query_times)
    odom_x, odom_y, odom_yaw = interpolate_pose(*map_to_odom, query_times)
    return compose(odom_x, odom_y, odom_yaw, robot_x, robot_y)


def distance_stats(distances) -> dict:
    """Minimum, 5th percentile and mean of the human-robot distances."""
    d = np.asarray(distances, dtype=float)
    if d.size == 0:
        return {"min_distance": None, "p5_distance": None, "mean_distance": None, "samples": 0}
    return {
        "min_distance": float(d.min()),
        "p5_distance": float(np.percentile(d, 5)),
        "mean_distance": float(d.mean()),
        "samples": int(d.size),
    }


def path_length(xs, ys) -> float:
    """Length of the polyline through the odometry positions."""
    xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    if xs.size < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))


def mean_or_none(values):
    values = np.asarray(values, dtype=float)
    return float(values.mean()) if values.size else None


def yaw_from_quaternion(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def mean_and_std(values) -> tuple[float | None, float | None]:
    """Mean and sample standard deviation (n - 1); std is None for fewer than 2 values."""
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return None, None
    return float(values.mean()), (float(values.std(ddof=1)) if values.size > 1 else None)
