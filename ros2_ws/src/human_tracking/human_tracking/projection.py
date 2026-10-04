"""Pinhole back-projection of a pixel with depth to a 3D point. No ROS imports."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, k) -> "Intrinsics":
        """From the row-major 3x3 ``K`` matrix of a ``sensor_msgs/CameraInfo``."""
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


def project_pixel(u: float, v: float, z: float, k: Intrinsics) -> tuple[float, float, float]:
    """Pixel ``(u, v)`` at depth ``z`` -> ``(X, Y, Z)`` in the camera optical frame.

    Optical frame convention: X right, Y down, Z forward.
    """
    if k.fx == 0.0 or k.fy == 0.0:
        raise ValueError("focal lengths must be non-zero")
    return ((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z)
