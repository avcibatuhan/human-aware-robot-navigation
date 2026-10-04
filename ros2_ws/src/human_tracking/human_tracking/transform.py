"""Rigid transform of a point by a translation and a unit quaternion. No ROS imports."""

Point = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w


def rotate(q: Quaternion, p: Point) -> Point:
    """Rotate ``p`` by the unit quaternion ``q`` (x, y, z, w)."""
    x, y, z, w = q
    px, py, pz = p
    # t = 2 * cross(q.xyz, p);  p' = p + w * t + cross(q.xyz, t)
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    )


def apply_transform(translation: Point, rotation: Quaternion, point: Point) -> Point:
    """Express ``point`` in the target frame: ``R * point + t``."""
    rx, ry, rz = rotate(rotation, point)
    return (rx + translation[0], ry + translation[1], rz + translation[2])
