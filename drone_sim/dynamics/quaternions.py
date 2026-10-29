"""Minimal Hamilton quaternion type used across the package.

Quaternions are stored scalar-first, ``q = w + x i + y j + z k``, and follow the
Hamilton product convention (``i j = k``). A *unit* quaternion represents a
rotation; :meth:`Quaternion.rotate` applies it to a 3D vector with the
sandwich product ``q v q*``.

Example:
    >>> import numpy as np
    >>> quarter_turn_z = Quaternion.from_axis_angle(np.array([0.0, 0.0, 1.0]), 90.0)
    >>> rotated = quarter_turn_z.rotate([1.0, 0.0, 0.0])
    >>> np.round([rotated.x, rotated.y, rotated.z], 12).tolist()
    [0.0, 1.0, 0.0]
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike


class Quaternion:
    """A quaternion ``w + x i + y j + z k`` (scalar first).

    Args:
        w: Scalar (real) part.
        x: Coefficient of ``i``.
        y: Coefficient of ``j``.
        z: Coefficient of ``k``.

    Attributes:
        w: Scalar (real) part.
        x: Coefficient of ``i``.
        y: Coefficient of ``j``.
        z: Coefficient of ``k``.
    """

    def __init__(self, w: float, x: float, y: float, z: float) -> None:
        self.w, self.x, self.y, self.z = w, x, y, z

    def __repr__(self) -> str:
        return f"Quaternion(w={self.w!r}, x={self.x!r}, y={self.y!r}, z={self.z!r})"

    def __mul__(self, other: Quaternion) -> Quaternion:
        """Return the Hamilton product ``self * other``.

        The product is not commutative: ``p * q`` composes the rotation ``q``
        first, then ``p``.

        Example:
            >>> i = Quaternion(0, 1, 0, 0)
            >>> j = Quaternion(0, 0, 1, 0)
            >>> i * j
            Quaternion(w=0, x=0, y=0, z=1)
        """
        w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quaternion(w, x, y, z)

    def conjugate(self) -> Quaternion:
        """Return the conjugate ``w - x i - y j - z k``.

        For a unit quaternion the conjugate is also the inverse rotation.
        """
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def rotate(self, vector: Sequence[float] | np.ndarray) -> Quaternion:
        """Rotate a 3D vector with the sandwich product ``q v q*``.

        Args:
            vector: Vector ``(vx, vy, vz)`` in the source frame.

        Returns:
            Pure quaternion ``0 + vx' i + vy' j + vz' k`` holding the rotated
            vector. ``self`` must be a unit quaternion for the result to be a
            pure rotation.
        """
        vx, vy, vz = vector
        vector_quaternion = Quaternion(0, vx, vy, vz)
        return self * vector_quaternion * self.conjugate()

    def norm(self) -> float:
        """Return the Euclidean norm ``sqrt(w² + x² + y² + z²)``."""
        return float(np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2))

    def normalize(self) -> Quaternion:
        """Return a unit-length copy of this quaternion.

        A zero quaternion has no direction, so it is returned unchanged.
        """
        length = self.norm()
        if length == 0:
            return Quaternion(self.w, self.x, self.y, self.z)
        return Quaternion(self.w / length, self.x / length,
                          self.y / length, self.z / length)

    def to_array(self) -> np.ndarray:
        """Return ``[w, x, y, z]`` as a float NumPy array."""
        return np.array([self.w, self.x, self.y, self.z], dtype=float)

    @classmethod
    def from_array(cls, values: ArrayLike) -> Quaternion:
        """Build a quaternion from a scalar-first ``[w, x, y, z]`` array."""
        w, x, y, z = np.asarray(values, dtype=float)
        return cls(w, x, y, z)

    @classmethod
    def from_axis_angle(cls, axis: np.ndarray, angle_deg: float) -> Quaternion:
        """Build the rotation of ``angle_deg`` degrees about ``axis``.

        Args:
            axis: Rotation axis as a **unit** 3-vector (NumPy array).
            angle_deg: Right-handed rotation angle in degrees.

        Returns:
            ``cos(θ/2) + sin(θ/2) (axis · [i, j, k])``.
        """
        half_angle_rad = np.deg2rad(angle_deg / 2)
        scalar_part = np.cos(half_angle_rad)
        vector_part = np.sin(half_angle_rad) * axis
        return Quaternion(scalar_part, *vector_part)

    def display(self) -> None:
        """Print the four components on one line."""
        print(f"w: {self.w}, x: {self.x}, y: {self.y}, z: {self.z}")
