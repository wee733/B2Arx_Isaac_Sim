# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Configured B2ARX TF/odometry seams and visualization.

No sensor or navigation algorithm lives here. The static ZED mount and the
odometry lever-arm correction are deliberately built from the same YAML value.
"""

import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


# Robot geometry:
#   B2 base -> R5a mount                    = (+0.220, 0, +0.100)
#   R5a mount -> embedded ZED USD root      = (+0.280, 0, -0.020)
#   USD root -> left camera                 = (+0.015, +0.060, +0.015)
#   zed_wrapper camera_link -> left camera  = (-0.010, +0.060, +0.016)
# The wrapper owns odom -> zed_camera_link. The inverse camera_link ->
# base_link transform and the Hesai mount are loaded from platform_adapters.yaml.


def _mapping(value, label: str) -> dict:
    if not isinstance(value, dict):
        raise RuntimeError(f"{label} must be a mapping")
    return dict(value)


def _frame(value, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f"{label} must be a non-empty frame name")
    return value.strip()


def _vector(value, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise RuntimeError(f"{label} must contain {length} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise RuntimeError(f"{label} must contain finite values")
    return result


def _mount(config: dict, name: str) -> dict:
    mount = _mapping(config.get(name), f"mounts.{name}")
    allowed = {"parent_frame", "child_frame", "translation", "quaternion_xyzw"}
    unknown = sorted(set(mount) - allowed)
    if unknown:
        raise RuntimeError(f"Unknown mounts.{name} keys: {', '.join(unknown)}")
    result = {
        "parent_frame": _frame(mount.get("parent_frame"), f"mounts.{name}.parent_frame"),
        "child_frame": _frame(mount.get("child_frame"), f"mounts.{name}.child_frame"),
        "translation": _vector(mount.get("translation"), 3, f"mounts.{name}.translation"),
        "quaternion": _vector(
            mount.get("quaternion_xyzw"), 4, f"mounts.{name}.quaternion_xyzw"
        ),
    }
    quaternion_norm = math.sqrt(sum(value * value for value in result["quaternion"]))
    if not math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
        raise RuntimeError(f"mounts.{name}.quaternion_xyzw must be unit length")
    return result


def _static_transform(
    *,
    name: str,
    parent: str,
    child: str,
    translation: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
    use_sim_time,
    condition,
) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        condition=condition,
        arguments=[
            "--x",
            str(translation[0]),
            "--y",
            str(translation[1]),
            "--z",
            str(translation[2]),
            "--qx",
            str(quaternion[0]),
            "--qy",
            str(quaternion[1]),
            "--qz",
            str(quaternion[2]),
            "--qw",
            str(quaternion[3]),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )


def _configured_platform_actions(context, *, bringup_share: str):
    config_path = Path(
        LaunchConfiguration("platform_config_file").perform(context)
    ).expanduser()
    if not config_path.is_file():
        raise RuntimeError(f"Platform adapter config does not exist: {config_path}")
    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config = _mapping(loaded, "platform adapter config")
    unknown = sorted(set(config) - {"mounts", "odometry_adapter"})
    if unknown:
        raise RuntimeError(f"Unknown platform adapter keys: {', '.join(unknown)}")

    mounts = _mapping(config.get("mounts"), "mounts")
    zed_mount = _mount(mounts, "zed_camera_to_base")
    hesai_mount = _mount(mounts, "base_to_hesai_lidar")
    odometry = _mapping(config.get("odometry_adapter"), "odometry_adapter")
    odometry_parameters = _mapping(
        odometry.get("ros__parameters"), "odometry_adapter.ros__parameters"
    )
    # Frames and geometry are injected from the mount instead of duplicated in
    # a second parameter block. Changing the YAML mount therefore updates both
    # the TF tree and the rigid-body odometry correction atomically.
    odometry_parameters.update(
        {
            "expected_camera_child_frame_id": zed_mount["parent_frame"],
            "base_child_frame_id": zed_mount["child_frame"],
            "camera_to_base_translation": list(zed_mount["translation"]),
            "camera_to_base_quaternion": list(zed_mount["quaternion"]),
        }
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_to_base = _static_transform(
        name="zed_camera_to_b2_base",
        parent=zed_mount["parent_frame"],
        child=zed_mount["child_frame"],
        translation=zed_mount["translation"],
        quaternion=zed_mount["quaternion"],
        use_sim_time=use_sim_time,
        condition=IfCondition(LaunchConfiguration("publish_zed_mount_tf")),
    )
    base_to_lidar = _static_transform(
        name="b2_base_to_hesai_lidar",
        parent=hesai_mount["parent_frame"],
        child=hesai_mount["child_frame"],
        translation=hesai_mount["translation"],
        quaternion=hesai_mount["quaternion"],
        use_sim_time=use_sim_time,
        condition=IfCondition(LaunchConfiguration("publish_hesai_tf")),
    )
    odometry_adapter = Node(
        package="b2arx_nav2_bringup",
        executable="odometry_adapter",
        name="odometry_adapter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_odometry_adapter")),
        parameters=[odometry_parameters, {"use_sim_time": use_sim_time}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=[
            "-d",
            os.path.join(bringup_share, "rviz", "b2arx_nvblox_nav2.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    return [camera_to_base, base_to_lidar, odometry_adapter, rviz]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_odometry_adapter", default_value="true"),
            DeclareLaunchArgument("publish_zed_mount_tf", default_value="true"),
            DeclareLaunchArgument("publish_hesai_tf", default_value="true"),
            DeclareLaunchArgument(
                "platform_config_file",
                default_value=os.path.join(
                    bringup_share, "config", "platform_adapters.yaml"
                ),
                description=(
                    "B2ARX frame, sensor mount, and odometry adapter contract."
                ),
            ),
            OpaqueFunction(
                function=_configured_platform_actions,
                kwargs={"bringup_share": bringup_share},
            ),
        ]
    )
