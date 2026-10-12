# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""B2ARX parameters around the official Isaac ROS Nvblox launch."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node, SetParameter, SetParametersFromFile

from b2arx_nav2_bringup.launch_common import (
    NAV2_PARAMS_FILE,
    include_launch,
    launch_config_bool,
    nvblox_container_name,
    package_file,
    project_file,
)


NVBLOX_ZED_LIDAR_ERROR = (
    "use_lidar_in_nvblox:=true is not supported by the Isaac ROS 4.5 ZED "
    "pipeline (the official launch rejects ZED + LiDAR fusion). Keep XT32 on "
    "/lidar_points as an independent PointCloud2 source, or launch a separate "
    "LiDAR-specialized Nvblox instance."
)


def _validate_sensor_contract(context):
    if not launch_config_bool(context, "start_nvblox"):
        return []
    if launch_config_bool(context, "use_lidar_in_nvblox"):
        raise RuntimeError(NVBLOX_ZED_LIDAR_ERROR)
    return []


def generate_launch_description() -> LaunchDescription:
    start_nvblox = LaunchConfiguration("start_nvblox")
    use_sim_time = LaunchConfiguration("use_sim_time")
    container_name = LaunchConfiguration("container_name")
    container_type = LaunchConfiguration("container_type")
    container_params_file = LaunchConfiguration("container_params_file")
    log_level = LaunchConfiguration("log_level")
    zed_camera_model = LaunchConfiguration("camera_model")
    use_lidar = LaunchConfiguration("use_lidar_in_nvblox")
    nvblox_params_file = LaunchConfiguration("nvblox_params_file")

    official_nvblox = include_launch(
        package_file(
            "nvblox_examples_bringup", "launch", "perception", "nvblox.launch.py"
        ),
        {
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
        },
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
                "container_name", default_value=nvblox_container_name()
            ),
            DeclareLaunchArgument(
                "container_type",
                default_value="multithreaded",
                choices=["multithreaded", "isolated_multithreaded"],
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "container_params_file",
                default_value=project_file("config", NAV2_PARAMS_FILE),
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
                default_value=project_file("config", "nvblox_b2arx.yaml"),
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
