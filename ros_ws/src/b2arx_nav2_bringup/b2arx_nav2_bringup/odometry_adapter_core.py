from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

DEFAULT_CAMERA_TO_BASE_TRANSLATION: Vector3 = (-0.525, 0.0, -0.079)
DEFAULT_CAMERA_TO_BASE_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class OdometryState:
    """ROS-independent pose and body-frame twist for one odometry sample."""

    position: Vector3
    orientation_xyzw: Quaternion
    linear_velocity: Vector3
    angular_velocity: Vector3


def _vector3(values: Sequence[float], name: str) -> Vector3:
    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError(f"{name} must contain 3 values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _unit_quaternion(values: Sequence[float], name: str) -> Quaternion:
    result = tuple(float(value) for value in values)
    if len(result) != 4:
        raise ValueError(f"{name} must contain 4 xyzw values, got {len(result)}")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")

    norm = math.sqrt(sum(value * value for value in result))
    if norm <= 1.0e-12:
        raise ValueError(f"{name} must not be a zero quaternion")
    return (
        result[0] / norm,
        result[1] / norm,
        result[2] / norm,
        result[3] / norm,
    )


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> Quaternion:
    """Compose two active rotations represented as xyzw quaternions."""

    lx, ly, lz, lw = _unit_quaternion(left, "left quaternion")
    rx, ry, rz, rw = _unit_quaternion(right, "right quaternion")
    return _unit_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ),
        "composed quaternion",
    )


def quaternion_conjugate(quaternion: Sequence[float]) -> Quaternion:
    x, y, z, w = _unit_quaternion(quaternion, "quaternion")
    return (-x, -y, -z, w)


def rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> Vector3:
    """Rotate a vector without constructing temporary quaternion objects."""

    qx, qy, qz, qw = _unit_quaternion(quaternion, "rotation quaternion")
    vx, vy, vz = _vector3(vector, "vector")

    # Unit-quaternion form of q * [v, 0] * conjugate(q).
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def cross(left: Sequence[float], right: Sequence[float]) -> Vector3:
    lx, ly, lz = _vector3(left, "left vector")
    rx, ry, rz = _vector3(right, "right vector")
    return (ly * rz - lz * ry, lz * rx - lx * rz, lx * ry - ly * rx)


def transform_camera_odometry_to_base(
    camera_state: OdometryState,
    camera_to_base_translation: Sequence[float] = DEFAULT_CAMERA_TO_BASE_TRANSLATION,
    camera_to_base_quaternion: Sequence[float] = DEFAULT_CAMERA_TO_BASE_QUATERNION,
) -> OdometryState:
    """Move ZED camera odometry to the B2 base origin.

    The fixed transform follows the standard ``T_parent_child`` convention:
    ``T_camera_base`` locates ``base_link`` in ``zed_camera_link``. Input
    twist is the camera-origin velocity expressed in the camera frame, as
    required by ``nav_msgs/Odometry``. The output twist applies the rigid-body
    lever arm and is re-expressed in ``base_link``.
    """

    camera_position = _vector3(camera_state.position, "camera position")
    odom_to_camera_rotation = _unit_quaternion(
        camera_state.orientation_xyzw,
        "camera orientation",
    )
    camera_linear_velocity = _vector3(
        camera_state.linear_velocity,
        "camera linear velocity",
    )
    camera_angular_velocity = _vector3(
        camera_state.angular_velocity,
        "camera angular velocity",
    )
    camera_to_base_offset = _vector3(
        camera_to_base_translation,
        "camera-to-base translation",
    )
    camera_to_base_rotation = _unit_quaternion(
        camera_to_base_quaternion,
        "camera-to-base quaternion",
    )

    rotated_offset = rotate_vector(odom_to_camera_rotation, camera_to_base_offset)
    base_position = (
        camera_position[0] + rotated_offset[0],
        camera_position[1] + rotated_offset[1],
        camera_position[2] + rotated_offset[2],
    )
    base_orientation = quaternion_multiply(
        odom_to_camera_rotation,
        camera_to_base_rotation,
    )

    lever_velocity = cross(camera_angular_velocity, camera_to_base_offset)
    base_origin_velocity_in_camera = (
        camera_linear_velocity[0] + lever_velocity[0],
        camera_linear_velocity[1] + lever_velocity[1],
        camera_linear_velocity[2] + lever_velocity[2],
    )
    base_to_camera_rotation = quaternion_conjugate(camera_to_base_rotation)
    base_linear_velocity = rotate_vector(
        base_to_camera_rotation,
        base_origin_velocity_in_camera,
    )
    base_angular_velocity = rotate_vector(
        base_to_camera_rotation,
        camera_angular_velocity,
    )

    return OdometryState(
        position=base_position,
        orientation_xyzw=base_orientation,
        linear_velocity=base_linear_velocity,
        angular_velocity=base_angular_velocity,
    )
