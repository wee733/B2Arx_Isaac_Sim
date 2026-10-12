# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Thin sim/real adapter around the official Stereolabs ZED launch file."""

from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from b2arx_nav2_bringup.launch_common import (
    include_launch,
    is_sim_flag,
    package_file,
    zed_arguments,
)


def generate_launch_description() -> LaunchDescription:
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
    official_zed = include_launch(
        package_file("zed_wrapper", "launch", "zed_camera.launch.py"),
        {
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
            "sim_mode": is_sim_flag(sensor_mode),
            "sim_address": sim_address,
            "sim_port": sim_port,
            # Keep the validated ROS Image path selectable without forking the
            # upstream ZED component or launch implementation.
            "param_overrides": [
                "debug.disable_nitros:=",
                disable_zed_nitros,
            ],
        },
        condition=IfCondition(start_zed_wrapper),
    )

    return LaunchDescription(
        [
            *zed_arguments(),
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
