"""Small replacements for the ROS 1 ``tf.transformations`` helpers used by the node (planar case)."""
import math
from typing import Sequence, Tuple


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Quaternion ``(x, y, z, w)`` of a rotation of ``yaw`` radians about the z axis."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q: Sequence[float]) -> float:
    """Yaw (rotation about z) of the quaternion ``(x, y, z, w)``."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
