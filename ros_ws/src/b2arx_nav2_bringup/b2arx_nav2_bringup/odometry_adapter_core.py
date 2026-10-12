from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

DEFAULT_CAMERA_TO_BASE_TRANSLATION: Vector3 = (-0.525, 0.0, -0.079)
DEFAULT_CAMERA_TO_BASE_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)

UNIT_QUATERNION_TOLERANCE = 1.0e-6


@dataclass(frozen=True)
class OdometryState:
    """ROS-independent pose and body-frame twist for one odometry sample."""

    position: Vector3
    orientation_xyzw: Quaternion
    linear_velocity: Vector3
    angular_velocity: Vector3


def vector3(values: Sequence[float], name: str) -> Vector3:
    """Validate a finite 3-vector."""

    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError(f"{name} must contain 3 values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def unit_quaternion(
    values: Sequence[float],
    name: str,
    *,
    require_unit: bool = False,
) -> Quaternion:
    """Validate an xyzw quaternion and return it normalized.

    ``require_unit`` rejects inputs that are not already unit length (within
    ``UNIT_QUATERNION_TOLERANCE``) instead of silently normalizing them. Use it
    for hand-written configuration, where a non-unit value is a typo.
    """

    result = tuple(float(value) for value in values)
    if len(result) != 4:
        raise ValueError(f"{name} must contain 4 xyzw values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")

    norm = math.sqrt(sum(value * value for value in result))
    if norm <= 1.0e-12:
        raise ValueError(f"{name} must not be a zero quaternion")
    if require_unit and not math.isclose(
        norm, 1.0, rel_tol=0.0, abs_tol=UNIT_QUATERNION_TOLERANCE
    ):
        raise ValueError(f"{name} must be unit length")
    return (
        result[0] / norm,
        result[1] / norm,
        result[2] / norm,
        result[3] / norm,
    )


# The underscored kernels below take already-validated tuples. They keep the
# per-message path free of repeated validation of constant mount values.


def _multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def _rotate(quaternion: Quaternion, vector: Vector3) -> Vector3:
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # Unit-quaternion form of q * [v, 0] * conjugate(q).
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _cross(left: Vector3, right: Vector3) -> Vector3:
    lx, ly, lz = left
    rx, ry, rz = right
    return (ly * rz - lz * ry, lz * rx - lx * rz, lx * ry - ly * rx)


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> Quaternion:
    """Compose two active rotations represented as xyzw quaternions."""

    return unit_quaternion(
        _multiply(
            unit_quaternion(left, "left quaternion"),
            unit_quaternion(right, "right quaternion"),
        ),
        "composed quaternion",
    )


def quaternion_conjugate(quaternion: Sequence[float]) -> Quaternion:
    return _conjugate(unit_quaternion(quaternion, "quaternion"))


def rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> Vector3:
    """Rotate a vector without constructing temporary quaternion objects."""

    return _rotate(
        unit_quaternion(quaternion, "rotation quaternion"),
        vector3(vector, "vector"),
    )


def cross(left: Sequence[float], right: Sequence[float]) -> Vector3:
    return _cross(vector3(left, "left vector"), vector3(right, "right vector"))


@dataclass(frozen=True)
class CameraToBaseMount:
    """Validated, precomputed ``T_camera_base`` rigid mount.

    Build it once with :meth:`from_values`; :meth:`apply` then validates only
    the incoming odometry sample.
    """

    translation: Vector3
    rotation: Quaternion
    inverse_rotation: Quaternion

    @classmethod
    def from_values(
        cls,
        translation: Sequence[float] = DEFAULT_CAMERA_TO_BASE_TRANSLATION,
        quaternion: Sequence[float] = DEFAULT_CAMERA_TO_BASE_QUATERNION,
    ) -> CameraToBaseMount:
        rotation = unit_quaternion(quaternion, "camera-to-base quaternion")
        return cls(
            translation=vector3(translation, "camera-to-base translation"),
            rotation=rotation,
            inverse_rotation=_conjugate(rotation),
        )

    def apply(self, camera_state: OdometryState) -> OdometryState:
        """Move ZED camera odometry to the B2 base origin.

        The fixed transform follows the standard ``T_parent_child``
        convention: ``T_camera_base`` locates ``base_link`` in
        ``zed_camera_link``. Input twist is the camera-origin velocity
        expressed in the camera frame, as required by ``nav_msgs/Odometry``.
        The output twist applies the rigid-body lever arm and is re-expressed
        in ``base_link``.
        """

        camera_position = vector3(camera_state.position, "camera position")
        odom_to_camera_rotation = unit_quaternion(
            camera_state.orientation_xyzw,
            "camera orientation",
        )
        camera_linear_velocity = vector3(
            camera_state.linear_velocity,
            "camera linear velocity",
        )
        camera_angular_velocity = vector3(
            camera_state.angular_velocity,
            "camera angular velocity",
        )

        rotated_offset = _rotate(odom_to_camera_rotation, self.translation)
        base_position = (
            camera_position[0] + rotated_offset[0],
            camera_position[1] + rotated_offset[1],
            camera_position[2] + rotated_offset[2],
        )
        base_orientation = unit_quaternion(
            _multiply(odom_to_camera_rotation, self.rotation),
            "composed quaternion",
        )

        lever_velocity = _cross(camera_angular_velocity, self.translation)
        base_origin_velocity_in_camera = (
            camera_linear_velocity[0] + lever_velocity[0],
            camera_linear_velocity[1] + lever_velocity[1],
            camera_linear_velocity[2] + lever_velocity[2],
        )
        return OdometryState(
            position=base_position,
            orientation_xyzw=base_orientation,
            linear_velocity=_rotate(
                self.inverse_rotation, base_origin_velocity_in_camera
            ),
            angular_velocity=_rotate(self.inverse_rotation, camera_angular_velocity),
        )


def transform_camera_odometry_to_base(
    camera_state: OdometryState,
    camera_to_base_translation: Sequence[float] = DEFAULT_CAMERA_TO_BASE_TRANSLATION,
    camera_to_base_quaternion: Sequence[float] = DEFAULT_CAMERA_TO_BASE_QUATERNION,
) -> OdometryState:
    """One-shot convenience wrapper around :class:`CameraToBaseMount`."""

    return CameraToBaseMount.from_values(
        camera_to_base_translation,
        camera_to_base_quaternion,
    ).apply(camera_state)
