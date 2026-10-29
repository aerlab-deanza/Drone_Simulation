"""Conversions between the plant frames and the filter frames.

The project uses two coordinate conventions:

* **Plant** (``dynamics`` and ``control``): world frame **NED** (North-East-Down)
  and body frame **FRD** (Forward-Right-Down).
* **Filters** (``estimation``) and the IMU stream: world frame with **z up**
  and body frame **FLU** (Forward-Left-Up).

Both pairs are related by the same matrix ``S = diag(1, -1, -1)``. ``S`` is a
proper rotation (180° about x, ``det S = +1``), so it maps right-handed frames
to right-handed frames. Vectors transform as ``v' = S v`` and scalar-first
quaternions as ``[w, x, -y, -z]``. ``S`` is its own inverse, so every
conversion here is also its own inverse.

Example:
    >>> ned_frd_vector_to_up_flu([1.0, 2.0, 3.0]).tolist()
    [1.0, -2.0, -3.0]
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

#: Frame-change matrix ``S = diag(1, -1, -1)`` (180° rotation about x).
FRAME_S: np.ndarray = np.diag([1.0, -1.0, -1.0])


def ned_frd_vector_to_up_flu(vector: ArrayLike) -> np.ndarray:
    """Convert a NED-world or FRD-body vector to the up-world/FLU-body frame.

    Args:
        vector: 3-vector in NED (world) or FRD (body) coordinates, any unit.

    Returns:
        The same vector in up-world or FLU-body coordinates, same unit.
    """
    return FRAME_S @ np.asarray(vector, dtype=float)


def up_flu_vector_to_ned_frd(vector: ArrayLike) -> np.ndarray:
    """Convert an up-world or FLU-body vector to the NED/FRD frame.

    Args:
        vector: 3-vector in up-world or FLU-body coordinates, any unit.

    Returns:
        The same vector in NED-world or FRD-body coordinates, same unit.
    """
    return FRAME_S @ np.asarray(vector, dtype=float)


def ned_frd_quaternion_to_up_flu(quaternion: ArrayLike) -> np.ndarray:
    """Convert an FRD-body→NED-world attitude to FLU-body→up-world.

    Args:
        quaternion: Scalar-first unit quaternion ``[w, x, y, z]``.

    Returns:
        ``[w, x, -y, -z]``, the equivalent attitude in the filter frames.

    Raises:
        ValueError: If ``quaternion`` does not have shape ``(4,)``.

    Example:
        >>> ned_frd_quaternion_to_up_flu([1.0, 0.1, 0.2, 0.3]).tolist()
        [1.0, 0.1, -0.2, -0.3]
    """
    q = np.asarray(quaternion, dtype=float).copy()
    if q.shape != (4,):
        raise ValueError("quaternion must have shape (4,)")
    return q * np.array([1.0, 1.0, -1.0, -1.0])


def up_flu_quaternion_to_ned_frd(quaternion: ArrayLike) -> np.ndarray:
    """Convert an FLU-body→up-world attitude to FRD-body→NED-world.

    Args:
        quaternion: Scalar-first unit quaternion ``[w, x, y, z]``.

    Returns:
        ``[w, x, -y, -z]``, the equivalent attitude in the plant frames.
    """
    return ned_frd_quaternion_to_up_flu(quaternion)


def ned_position_to_plot(position: ArrayLike) -> np.ndarray:
    """Map a NED position to plotting coordinates ``(x, y, altitude)``.

    Args:
        position: ``[north, east, down]`` in metres.

    Returns:
        ``[north, east, up]`` in metres, so altitude is positive upward.
    """
    x, y, down = np.asarray(position, dtype=float)
    return np.array([x, y, -down])


def filter_position_to_plot(position: ArrayLike) -> np.ndarray:
    """Map a filter-frame position to plotting coordinates ``(x, y, altitude)``.

    The filter world frame is ``S`` applied to NED, i.e. ``(x, -y, up)``.

    Args:
        position: Filter-frame position ``[x, -y, up]`` in metres.

    Returns:
        ``[x, y, up]`` in metres, matching :func:`ned_position_to_plot`.
    """
    x, negative_y, up = np.asarray(position, dtype=float)
    return np.array([x, -negative_y, up])
