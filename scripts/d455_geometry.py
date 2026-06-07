from __future__ import annotations

import numpy as np


D455_USD_RELATIVE_PATH = "/Sensors/Intel/RealSense/rsd455.usd"
"""Official Isaac Sim RealSense D455 asset path relative to ``ISAAC_NUCLEUS_DIR``.

The current Isaac 5.1 asset bucket stores it under:
Isaac > Sensors > Intel > RealSense > rsd455.usd.
"""


def resolve_d455_usd_path(isaac_nucleus_dir: str) -> str:
    return isaac_nucleus_dir.rstrip("/") + D455_USD_RELATIVE_PATH


# Wrist-mounted sensor pose relative to R5a_link6: camera body in front of the wrist and pitched 30 degrees down.
D455_MOUNT_POS = np.array([0.06, 0.0, 0.13], dtype=np.float64)
D455_MOUNT_RPY = (0.0, np.deg2rad(30.0), 0.0)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _quat_wxyz_from_matrix(rot: np.ndarray) -> tuple[float, float, float, float]:
    trace = np.trace(rot)
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rot[2, 1] - rot[1, 2]) / scale
        y = (rot[0, 2] - rot[2, 0]) / scale
        z = (rot[1, 0] - rot[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rot)))
        if axis == 0:
            scale = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
            w = (rot[2, 1] - rot[1, 2]) / scale
            x = 0.25 * scale
            y = (rot[0, 1] + rot[1, 0]) / scale
            z = (rot[0, 2] + rot[2, 0]) / scale
        elif axis == 1:
            scale = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
            w = (rot[0, 2] - rot[2, 0]) / scale
            x = (rot[0, 1] + rot[1, 0]) / scale
            y = 0.25 * scale
            z = (rot[1, 2] + rot[2, 1]) / scale
        else:
            scale = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
            w = (rot[1, 0] - rot[0, 1]) / scale
            x = (rot[0, 2] + rot[2, 0]) / scale
            y = (rot[1, 2] + rot[2, 1]) / scale
            z = 0.25 * scale

    quat = np.array([w, x, y, z], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    if quat[0] < 0.0:
        quat *= -1.0
    return tuple(float(v) for v in quat)


D455_MOUNT_ROT_MATRIX = _rpy_matrix(*D455_MOUNT_RPY)
D455_MOUNT_ROT = _quat_wxyz_from_matrix(D455_MOUNT_ROT_MATRIX)

# Isaac's official D455 USD contains these Camera prim names.
D455_OFFICIAL_CAMERA_PRIMS = {
    "depth": "Camera_Pseudo_Depth",
    "color": "Camera_OmniVision_OV9782_Color",
    "infra1": "Camera_OmniVision_OV9782_Left",
    "infra2": "Camera_OmniVision_OV9782_Right",
}

# IsaacLab binds one CameraCfg to each official camera prim, so downstream code can read tensors through
# scene["d455_*_camera"] while the visual model and camera layout stay inside the official D455 USD.
D455_IMAGE_WIDTH = 640
D455_IMAGE_HEIGHT = 480
