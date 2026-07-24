#!/usr/bin/env python3
"""Translate a portable simulation profile into the existing scene CLI.

This module is intentionally independent of Isaac Sim.  It owns only the
configuration contract and delegates all simulation, sensor, and policy work
to :mod:`isaac_b2arx_scene` and its existing helpers.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import yaml

try:
    from .zed_isaac_sim import (
        ZED_STREAM_FPS,
        ZED_STREAM_PORT,
        ZED_STREAM_RESOLUTION,
        ZED_STREAM_TRANSPORT,
        validate_zed_stream_settings,
    )
except ImportError:  # Direct execution: ``python scripts/simulation_config.py``.
    from zed_isaac_sim import (  # type: ignore[no-redef]
        ZED_STREAM_FPS,
        ZED_STREAM_PORT,
        ZED_STREAM_RESOLUTION,
        ZED_STREAM_TRANSPORT,
        validate_zed_stream_settings,
    )


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_USD = PROJECT_ROOT / "assets" / "my_B2Arx" / "my_b2arx" / "my_robot.usd"
DEFAULT_POLICY_PROFILE = PROJECT_ROOT / "config" / "policies" / "basic_locomotion.yaml"
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "simulation" / "warehouse.yaml"

SCENE_ASSETS = frozenset(
    ("default", "minimal", "grid", "rough_plane", "warehouse", "warehouse_local", "hospital")
)
CONTROL_MODES = frozenset(("hold", "policy"))
ARM_GAIN_PROFILES = frozenset(("identified", "train"))
VIEWER_CAMERAS = frozenset(("scene", "zed", "zed_left", "zed_right", "wrist"))

ROOT_KEYS = frozenset(
    (
        "schema_version",
        "robot_usd",
        "environment",
        "zed",
        "wrist_camera",
        "xt32",
        "ros",
        "policy",
        "viewer",
        "simulation",
    )
)
SECTION_KEYS = {
    "environment": frozenset(("scene_asset", "usd", "workspace")),
    "zed": frozenset(
        ("enabled", "sdk_stream", "save_frames", "resolution", "fps", "port", "transport", "print_debug")
    ),
    "wrist_camera": frozenset(
        (
            "enabled",
            "camera_prim_path",
            "width",
            "height",
            "frame_skip_count",
            "frame_id",
            "color_topic",
            "color_info_topic",
            "aligned_depth_topic",
            "aligned_depth_info_topic",
        )
    ),
    "xt32": frozenset(("enabled", "topic", "frame_id", "show_debug_view")),
    "ros": frozenset(("enabled", "domain_id", "nav2")),
    "policy": frozenset(("control_mode", "profile", "arm_gain_profile", "print_debug")),
    "viewer": frozenset(("camera", "headless")),
    "simulation": frozenset(
        ("num_envs", "env_spacing", "duration", "device", "disable_fabric")
    ),
}


class SimulationConfigError(ValueError):
    """Raised when a simulation YAML does not match the launcher contract."""


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise SimulationConfigError(f"{label} must be a mapping")
    return dict(value)


def _reject_unknown_keys(values: Mapping[str, Any], allowed: frozenset[str], label: str) -> None:
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise SimulationConfigError(f"Unknown {label} keys: {', '.join(unknown)}")


def _bool(values: Mapping[str, Any], key: str, default: bool, label: str) -> bool:
    value = values.get(key, default)
    if not isinstance(value, bool):
        raise SimulationConfigError(f"{label}.{key} must be true or false")
    return value


def _int(values: Mapping[str, Any], key: str, default: int, label: str) -> int:
    value = values.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise SimulationConfigError(f"{label}.{key} must be an integer")
    return value


def _float(values: Mapping[str, Any], key: str, default: float, label: str) -> float:
    value = values.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SimulationConfigError(f"{label}.{key} must be a number")
    return float(value)


def _string(values: Mapping[str, Any], key: str, default: str, label: str) -> str:
    value = values.get(key, default)
    if not isinstance(value, str) or not value.strip():
        raise SimulationConfigError(f"{label}.{key} must be a non-empty string")
    return value.strip()


def _choice(value: str, allowed: frozenset[str], label: str) -> str:
    if value not in allowed:
        choices = ", ".join(sorted(allowed))
        raise SimulationConfigError(f"{label} must be one of: {choices}; got {value!r}")
    return value


def _resolve_local_path(value: Any, config_dir: Path, label: str) -> str:
    if not isinstance(value, (str, Path)) or not str(value).strip():
        raise SimulationConfigError(f"{label} must be a non-empty local path")
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = config_dir / path
    return str(path.resolve())


def _resolve_path_or_url(value: Any, config_dir: Path, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SimulationConfigError(f"{label} must be a non-empty path or URL")
    text = value.strip()
    if text.startswith(("http://", "https://", "omniverse://")):
        return text
    return _resolve_local_path(text, config_dir, label)


def load_simulation_profile(config_path: str | Path) -> tuple[Path, dict[str, Any]]:
    """Load and structurally validate a simulation profile."""
    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Simulation config not found: {path}")
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    profile = _mapping(loaded, "simulation config")
    _reject_unknown_keys(profile, ROOT_KEYS, "top-level")
    schema_version = profile.get("schema_version", 1)
    if schema_version != 1:
        raise SimulationConfigError(f"Unsupported simulation config schema_version: {schema_version!r}")
    for section_name, allowed_keys in SECTION_KEYS.items():
        section = _mapping(profile.get(section_name), section_name)
        _reject_unknown_keys(section, allowed_keys, section_name)
        profile[section_name] = section
    return path, profile


def config_arguments(config_path: str | Path) -> list[str]:
    """Return existing ``isaac_b2arx_scene.py`` arguments for one YAML profile."""
    path, profile = load_simulation_profile(config_path)
    config_dir = path.parent

    robot_usd = _resolve_local_path(
        profile.get("robot_usd", DEFAULT_ROBOT_USD), config_dir, "robot_usd"
    )

    environment = profile["environment"]
    scene_asset = _choice(
        _string(environment, "scene_asset", "default", "environment"),
        SCENE_ASSETS,
        "environment.scene_asset",
    )
    workspace = _bool(environment, "workspace", True, "environment")
    environment_usd_value = environment.get("usd")
    environment_usd = None
    if environment_usd_value not in (None, ""):
        environment_usd = _resolve_path_or_url(
            environment_usd_value, config_dir, "environment.usd"
        )

    zed = profile["zed"]
    zed_enabled = _bool(zed, "enabled", True, "zed")
    zed_sdk_stream = _bool(zed, "sdk_stream", True, "zed")
    zed_save_frames = _bool(zed, "save_frames", False, "zed")
    zed_resolution = _string(zed, "resolution", ZED_STREAM_RESOLUTION, "zed")
    zed_fps = _int(zed, "fps", ZED_STREAM_FPS, "zed")
    zed_port = _int(zed, "port", ZED_STREAM_PORT, "zed")
    zed_transport = _string(zed, "transport", ZED_STREAM_TRANSPORT, "zed").upper()
    zed_print_debug = _bool(zed, "print_debug", False, "zed")
    try:
        validate_zed_stream_settings(zed_resolution, zed_fps, zed_port, zed_transport)
    except ValueError as exc:
        raise SimulationConfigError(str(exc)) from exc
    if not zed_enabled and zed_sdk_stream:
        raise SimulationConfigError("zed.sdk_stream cannot be true when zed.enabled is false")
    if not zed_enabled and zed_save_frames:
        raise SimulationConfigError("zed.save_frames cannot be true when zed.enabled is false")

    wrist_camera = profile["wrist_camera"]
    wrist_camera_enabled = _bool(
        wrist_camera, "enabled", False, "wrist_camera"
    )
    wrist_camera_prim_path = _string(
        wrist_camera, "camera_prim_path", "auto", "wrist_camera"
    )
    wrist_camera_width = _int(wrist_camera, "width", 1280, "wrist_camera")
    wrist_camera_height = _int(wrist_camera, "height", 720, "wrist_camera")
    wrist_camera_frame_skip_count = _int(
        wrist_camera, "frame_skip_count", 0, "wrist_camera"
    )
    wrist_camera_frame_id = _string(
        wrist_camera,
        "frame_id",
        "wrist_camera_color_optical_frame",
        "wrist_camera",
    )
    wrist_camera_color_topic = _string(
        wrist_camera,
        "color_topic",
        "/wrist_camera/color/image_raw",
        "wrist_camera",
    )
    wrist_camera_color_info_topic = _string(
        wrist_camera,
        "color_info_topic",
        "/wrist_camera/color/camera_info",
        "wrist_camera",
    )
    wrist_camera_depth_topic = _string(
        wrist_camera,
        "aligned_depth_topic",
        "/wrist_camera/aligned_depth_to_color/image_raw",
        "wrist_camera",
    )
    wrist_camera_depth_info_topic = _string(
        wrist_camera,
        "aligned_depth_info_topic",
        "/wrist_camera/aligned_depth_to_color/camera_info",
        "wrist_camera",
    )
    if wrist_camera_width <= 0 or wrist_camera_height <= 0:
        raise SimulationConfigError("wrist_camera width and height must be positive")
    if wrist_camera_frame_skip_count < 0:
        raise SimulationConfigError(
            "wrist_camera.frame_skip_count must be non-negative"
        )
    if wrist_camera_frame_id.startswith("/"):
        raise SimulationConfigError(
            "wrist_camera.frame_id must not start with '/'"
        )
    wrist_camera_topics = (
        wrist_camera_color_topic,
        wrist_camera_color_info_topic,
        wrist_camera_depth_topic,
        wrist_camera_depth_info_topic,
    )
    if any(not topic.startswith("/") for topic in wrist_camera_topics):
        raise SimulationConfigError("all wrist_camera topics must be absolute")
    if len(set(wrist_camera_topics)) != len(wrist_camera_topics):
        raise SimulationConfigError("all wrist_camera topics must be unique")

    xt32 = profile["xt32"]
    xt32_enabled = _bool(xt32, "enabled", True, "xt32")
    xt32_topic = _string(xt32, "topic", "/lidar_points", "xt32")
    xt32_frame_id = _string(xt32, "frame_id", "hesai_lidar", "xt32")
    xt32_debug = _bool(xt32, "show_debug_view", False, "xt32")

    ros = profile["ros"]
    ros_enabled = _bool(ros, "enabled", False, "ros")
    ros_domain_id = _int(ros, "domain_id", 23, "ros")
    nav2_enabled = _bool(ros, "nav2", False, "ros")
    if not 0 <= ros_domain_id <= 232:
        raise SimulationConfigError("ros.domain_id must be between 0 and 232")
    if xt32_enabled and not ros_enabled:
        raise SimulationConfigError("xt32.enabled requires ros.enabled so PointCloud2 can be published")
    if wrist_camera_enabled and not ros_enabled:
        raise SimulationConfigError(
            "wrist_camera.enabled requires ros.enabled so RGB-D can be published"
        )

    policy = profile["policy"]
    control_mode = _choice(
        _string(policy, "control_mode", "hold", "policy"),
        CONTROL_MODES,
        "policy.control_mode",
    )
    policy_profile_value = policy.get("profile", DEFAULT_POLICY_PROFILE)
    policy_profile = _resolve_local_path(
        policy_profile_value, config_dir, "policy.profile"
    )
    arm_gain_profile = _choice(
        _string(policy, "arm_gain_profile", "identified", "policy"),
        ARM_GAIN_PROFILES,
        "policy.arm_gain_profile",
    )
    policy_print_debug = _bool(policy, "print_debug", False, "policy")

    viewer = profile["viewer"]
    viewer_camera = _choice(
        _string(viewer, "camera", "scene", "viewer"),
        VIEWER_CAMERAS,
        "viewer.camera",
    )
    headless = _bool(viewer, "headless", False, "viewer")

    simulation = profile["simulation"]
    num_envs = _int(simulation, "num_envs", 1, "simulation")
    env_spacing = _float(simulation, "env_spacing", 4.0, "simulation")
    duration = _float(simulation, "duration", 0.0, "simulation")
    device_value = simulation.get("device")
    device = None
    if device_value not in (None, ""):
        device = _string(simulation, "device", "", "simulation")
    disable_fabric = _bool(simulation, "disable_fabric", False, "simulation")
    if num_envs < 1:
        raise SimulationConfigError("simulation.num_envs must be at least 1")
    if env_spacing <= 0.0:
        raise SimulationConfigError("simulation.env_spacing must be positive")
    if duration < 0.0:
        raise SimulationConfigError("simulation.duration cannot be negative")

    if nav2_enabled:
        missing = []
        if not ros_enabled:
            missing.append("ros.enabled")
        if not zed_enabled:
            missing.append("zed.enabled")
        if not zed_sdk_stream:
            missing.append("zed.sdk_stream")
        if not xt32_enabled:
            missing.append("xt32.enabled")
        if control_mode != "policy":
            missing.append("policy.control_mode=policy")
        if missing:
            raise SimulationConfigError(
                "ros.nav2 requires " + ", ".join(missing)
            )

    arguments = [
        "--robot_usd",
        robot_usd,
        "--scene_asset",
        scene_asset,
        "--zed_stream_resolution",
        zed_resolution,
        "--zed_stream_fps",
        str(zed_fps),
        "--zed_stream_port",
        str(zed_port),
        "--zed_stream_transport",
        zed_transport,
        "--xt32_pointcloud_topic",
        xt32_topic,
        "--xt32_frame_id",
        xt32_frame_id,
        "--wrist_camera_prim_path",
        wrist_camera_prim_path,
        "--wrist_camera_width",
        str(wrist_camera_width),
        "--wrist_camera_height",
        str(wrist_camera_height),
        "--wrist_camera_frame_skip_count",
        str(wrist_camera_frame_skip_count),
        "--wrist_camera_frame_id",
        wrist_camera_frame_id,
        "--wrist_camera_color_topic",
        wrist_camera_color_topic,
        "--wrist_camera_color_info_topic",
        wrist_camera_color_info_topic,
        "--wrist_camera_aligned_depth_topic",
        wrist_camera_depth_topic,
        "--wrist_camera_aligned_depth_info_topic",
        wrist_camera_depth_info_topic,
        "--ros2_domain_id",
        str(ros_domain_id),
        "--control_mode",
        control_mode,
        "--deploy_config",
        policy_profile,
        "--arm_gain_profile",
        arm_gain_profile,
        "--viewer_camera",
        viewer_camera,
        "--num_envs",
        str(num_envs),
        "--env_spacing",
        str(env_spacing),
        "--duration",
        str(duration),
    ]
    if environment_usd is not None:
        arguments.extend(("--environment_usd", environment_usd))
    if not workspace:
        arguments.append("--no_workspace")
    if not zed_enabled:
        arguments.append("--no_zed")
    if not zed_sdk_stream:
        arguments.append("--no_zed_sdk_stream")
    if zed_save_frames:
        arguments.append("--save_zed_frames")
    if zed_print_debug:
        arguments.append("--print_zed_debug")
    if not xt32_enabled:
        arguments.append("--no_xt32_pointcloud")
    if xt32_debug:
        arguments.append("--show_xt32_debug_view")
    if wrist_camera_enabled:
        arguments.append("--wrist_camera")
    if ros_enabled:
        arguments.append("--ros2")
    if nav2_enabled:
        arguments.append("--nav2")
    if policy_print_debug:
        arguments.append("--print_policy_debug")
    if headless:
        arguments.append("--headless")
    if device is not None:
        arguments.extend(("--device", device))
    if disable_fabric:
        arguments.append("--disable_fabric")
    return arguments


def launch_arguments(config_path: str | Path, extra_arguments: Sequence[str] = ()) -> list[str]:
    """Build the final scene argv; user CLI is deliberately appended last."""
    return [*config_arguments(config_path), *extra_arguments]


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Simulation YAML profile.")
    parser.add_argument(
        "--emit",
        choices=("lines", "null"),
        default="lines",
        help="Argument serialization for the shell wrapper.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        arguments = config_arguments(args.config)
    except (FileNotFoundError, OSError, yaml.YAMLError, SimulationConfigError) as exc:
        print(f"simulation config error: {exc}", file=sys.stderr)
        return 2
    separator = "\0" if args.emit == "null" else "\n"
    sys.stdout.write(separator.join(arguments))
    if arguments:
        sys.stdout.write(separator)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
