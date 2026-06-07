from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DeploySphereLimits:
    radius: tuple[float, float] = (0.30, 0.60)
    pitch: tuple[float, float] = (-0.50, 1.00)
    yaw: tuple[float, float] = (-1.50, 1.50)


@dataclass(frozen=True)
class TargetSphereDebug:
    target_world: np.ndarray
    sphere_center_world: np.ndarray
    local_yaw_frame: np.ndarray
    sphere_raw: np.ndarray
    sphere_clamped: np.ndarray


DEPLOY_LIMITS = DeploySphereLimits()
SPHERE_CENTER_X_OFFSET = 0.23
SPHERE_CENTER_Z = 0.76


def _as_vec3(value: np.ndarray | tuple[float, float, float] | list[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,):
        raise ValueError(f"Expected a 3-vector, got shape {vector.shape}")
    return vector


def _yaw_rotation(yaw: float) -> np.ndarray:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def sphere_center_world(
    root_pos_world: np.ndarray | tuple[float, float, float] | list[float],
    yaw: float,
    *,
    x_offset: float = SPHERE_CENTER_X_OFFSET,
    z: float = SPHERE_CENTER_Z,
) -> np.ndarray:
    """Mirror the training sphere-center contract in world coordinates."""
    root = _as_vec3(root_pos_world)
    center = root.copy()
    center[2] = z
    center += _yaw_rotation(yaw) @ np.array([x_offset, 0.0, 0.0], dtype=np.float64)
    return center


def cart_to_sphere(cart_yaw_frame: np.ndarray | tuple[float, float, float] | list[float]) -> np.ndarray:
    """Convert yaw-frame Cartesian target to deployment `(radius, pitch, yaw)`."""
    cart = _as_vec3(cart_yaw_frame)
    radius = max(float(np.linalg.norm(cart)), 1e-4)
    xy = float(np.linalg.norm(cart[:2]))
    pitch = math.atan2(float(cart[2]), xy)
    yaw = math.atan2(float(cart[1]), float(cart[0]))
    return np.array([radius, pitch, yaw], dtype=np.float64)


def clamp_sphere(
    sphere: np.ndarray | tuple[float, float, float] | list[float],
    limits: DeploySphereLimits = DEPLOY_LIMITS,
) -> np.ndarray:
    """Clamp an EE sphere command to the deploy-side command buffer ranges."""
    command = _as_vec3(sphere)
    return np.array(
        [
            np.clip(command[0], limits.radius[0], limits.radius[1]),
            np.clip(command[1], limits.pitch[0], limits.pitch[1]),
            np.clip(command[2], limits.yaw[0], limits.yaw[1]),
        ],
        dtype=np.float64,
    )


def target_world_to_sphere(
    target_world: np.ndarray | tuple[float, float, float] | list[float],
    root_pos_world: np.ndarray | tuple[float, float, float] | list[float],
    yaw: float,
    *,
    limits: DeploySphereLimits = DEPLOY_LIMITS,
) -> TargetSphereDebug:
    """Convert a world-frame target point into the policy's deploy EE command."""
    target = _as_vec3(target_world)
    center = sphere_center_world(root_pos_world, yaw)
    local_yaw_frame = _yaw_rotation(yaw).T @ (target - center)
    sphere_raw = cart_to_sphere(local_yaw_frame)
    sphere_clamped = clamp_sphere(sphere_raw, limits)
    return TargetSphereDebug(
        target_world=target,
        sphere_center_world=center,
        local_yaw_frame=local_yaw_frame,
        sphere_raw=sphere_raw,
        sphere_clamped=sphere_clamped,
    )
