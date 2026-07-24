# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Thin sim/real adapter around the official Stereolabs ZED launch file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    zed_share = get_package_share_directory("zed_wrapper")

    sensor_mode = LaunchConfiguration("sensor_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_address = LaunchConfiguration("sim_address")
    sim_port = LaunchConfiguration("sim_port")
    serial_number = LaunchConfiguration("serial_number")
    camera_model = LaunchConfiguration("camera_model")
    zed_params_file = LaunchConfiguration("zed_params_file")
    disable_zed_nitros = LaunchConfiguration("disable_zed_nitros")
    start_zed_wrapper = LaunchConfiguration("start_zed_wrapper")

    # zed_wrapper owns camera acquisition, decoding, rectification, depth,
    # odometry, and the camera's internal TF tree. This project only selects
    # the data source and supplies robot-specific parameter overrides.
    official_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_share, "launch", "zed_camera.launch.py")
        ),
        condition=IfCondition(start_zed_wrapper),
        launch_arguments={
            "camera_name": "zed",
            "camera_model": camera_model,
            "node_name": "zed_node",
            # An empty container name deliberately lets the official launch
            # create and manage /zed/zed_container.
            "container_name": "",
            "ros_params_override_path": zed_params_file,
            "serial_number": serial_number,
            "publish_urdf": "true",
            "publish_tf": "true",
            "publish_map_tf": "true",
            "publish_imu_tf": "false",
            "enable_ipc": "false",
            "use_sim_time": use_sim_time,
            "sim_mode": PythonExpression(
                ["'true' if '", sensor_mode, "' == 'sim' else 'false'"]
            ),
            "sim_address": sim_address,
            "sim_port": sim_port,
            # Keep the validated ROS Image path selectable without forking the
            # upstream ZED component or launch implementation.
            "param_overrides": [
                "debug.disable_nitros:=",
                disable_zed_nitros,
            ],
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_mode",
                default_value="sim",
                choices=["sim", "real"],
                description="Select the Isaac Sim stream or a physical ZED camera.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=PythonExpression(
                    ["'true' if '", sensor_mode, "' == 'sim' else 'false'"]
                ),
            ),
            DeclareLaunchArgument(
                "sim_address",
                default_value=EnvironmentVariable(
                    "SIM_ADDRESS", default_value="127.0.0.1"
                ),
                description=(
                    "Address of the Isaac Sim host that publishes the official "
                    "Stereolabs simulation stream."
                ),
            ),
            DeclareLaunchArgument(
                "sim_port",
                default_value=EnvironmentVariable("ZED_PORT", default_value="30000"),
            ),
            DeclareLaunchArgument("serial_number", default_value="0"),
            DeclareLaunchArgument("camera_model", default_value="zedx"),
            DeclareLaunchArgument("start_zed_wrapper", default_value="true"),
            DeclareLaunchArgument("disable_zed_nitros", default_value="true"),
            DeclareLaunchArgument(
                "zed_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "zedx_nvblox_release_4_5.yaml"
                ),
                description="B2ARX overrides layered onto the official ZED configs.",
            ),
            DeclareLaunchArgument(
                "zed_sdk_settings_path",
                default_value=EnvironmentVariable(
                    "ZED_SDK_SETTINGS_PATH", default_value=""
                ),
            ),
            GroupAction(
                actions=[
                    SetEnvironmentVariable(
                        "ZED_SDK_SETTINGS_PATH",
                        LaunchConfiguration("zed_sdk_settings_path"),
                    ),
                    official_zed,
                ]
            ),
        ]
    )
