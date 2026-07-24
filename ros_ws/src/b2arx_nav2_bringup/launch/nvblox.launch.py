# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""B2ARX parameters around the official Isaac ROS Nvblox launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


NVBLOX_ZED_LIDAR_ERROR = (
    "use_lidar_in_nvblox:=true is not supported by the Isaac ROS 4.5 ZED "
    "pipeline (the official launch rejects ZED + LiDAR fusion). Keep XT32 on "
    "/lidar_points as an independent PointCloud2 source, or launch a separate "
    "LiDAR-specialized Nvblox instance."
)


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _validate_sensor_contract(context):
    if not _is_true(LaunchConfiguration("start_nvblox").perform(context)):
        return []

    raw_value = LaunchConfiguration("use_lidar_in_nvblox").perform(context)
    normalized_value = raw_value.strip().lower()
    if _is_true(raw_value):
        raise RuntimeError(NVBLOX_ZED_LIDAR_ERROR)
    if normalized_value not in {"0", "false", "no", "off"}:
        raise ValueError(
            "use_lidar_in_nvblox must be a boolean value, got "
            f"{raw_value!r}."
        )
    return []


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    nvblox_examples_share = get_package_share_directory("nvblox_examples_bringup")

    start_nvblox = LaunchConfiguration("start_nvblox")
    use_sim_time = LaunchConfiguration("use_sim_time")
    container_name = LaunchConfiguration("container_name")
    container_type = LaunchConfiguration("container_type")
    container_params_file = LaunchConfiguration("container_params_file")
    log_level = LaunchConfiguration("log_level")
    zed_camera_model = LaunchConfiguration("camera_model")
    use_lidar = LaunchConfiguration("use_lidar_in_nvblox")
    nvblox_params_file = LaunchConfiguration("nvblox_params_file")

    official_nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nvblox_examples_share,
                "launch",
                "perception",
                "nvblox.launch.py",
            )
        ),
        launch_arguments={
            "mode": "static",
            "camera": zed_camera_model,
            "num_cameras": "1",
            "lidar": use_lidar,
            "container_name": container_name,
            # In the default path the official launch owns the one shared
            # component_container_mt. Nav2 subsequently loads its components
            # into this same container.
            # Isaac ROS ArgumentContainer uses Python eval() for primitive
            # conversion. Uppercase "True"/"False" are therefore required;
            # lowercase strings remain strings and are truthy in the official
            # `if args.run_standalone` check.
            "run_standalone": IfElseSubstitution(
                EqualsSubstitution(container_type, "multithreaded"),
                if_value="True",
                else_value="False",
            ),
        }.items(),
    )

    # The upstream launch exposes its standard multithreaded container but not
    # the isolated executor variant. Preserve the old A/B option here without
    # ever creating a second container.
    isolated_container = Node(
        package="rclcpp_components",
        executable="component_container_isolated",
        name=container_name,
        output="screen",
        arguments=[
            "--use_multi_threaded_executor",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        on_exit=Shutdown(),
        condition=IfCondition(
            EqualsSubstitution(container_type, "isolated_multithreaded")
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_nvblox", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "container_name", default_value=NVBLOX_CONTAINER_NAME
            ),
            DeclareLaunchArgument(
                "container_type",
                default_value="multithreaded",
                choices=["multithreaded", "isolated_multithreaded"],
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "container_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "b2arx_nav2.yaml"
                ),
                description=(
                    "Parameters injected into the sole shared container before "
                    "Nvblox and composed Nav2 are loaded."
                ),
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="zedx",
                choices=["zed2", "zedx"],
            ),
            DeclareLaunchArgument(
                "use_lidar_in_nvblox",
                default_value="false",
                description=(
                    "Must remain false for the official ZED specialization. "
                    "XT32 remains available independently on /lidar_points."
                ),
            ),
            DeclareLaunchArgument(
                "nvblox_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "nvblox_b2arx.yaml"
                ),
                description=(
                    "Robot-specific parameters applied in the scope of the "
                    "official Isaac ROS Nvblox launch."
                ),
            ),
            OpaqueFunction(function=_validate_sensor_contract),
            GroupAction(
                condition=IfCondition(start_nvblox),
                actions=[
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    SetParametersFromFile(container_params_file),
                    SetParametersFromFile(nvblox_params_file),
                    isolated_container,
                    official_nvblox,
                ],
            ),
        ]
    )
