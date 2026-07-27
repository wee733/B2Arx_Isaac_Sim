# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Simulation profile selecting depth- or XT32-backed Nav2 costmaps."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    navigation_mode = LaunchConfiguration("navigation_mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "navigation_mode",
                default_value="depth",
                choices=["depth", "lidar"],
                description=(
                    "depth: ZED + Isaac ROS Nvblox costmap; "
                    "lidar: XT32 PointCloud2 + Nav2 ObstacleLayer."
                ),
            ),
            DeclareLaunchArgument(
                "start_zed_wrapper",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Keep true while ZED VIO owns localization; disable only "
                    "when an external stack provides /b2/odom and map TF."
                ),
            ),
            DeclareLaunchArgument(
                "start_odometry_adapter",
                default_value="true",
                choices=["true", "false"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share, "launch", "isaac_ros_nav2.launch.py"
                    )
                ),
                launch_arguments={
                    "sensor_mode": "sim",
                    "use_sim_time": "true",
                    "start_zed_wrapper": LaunchConfiguration(
                        "start_zed_wrapper"
                    ),
                    "start_odometry_adapter": LaunchConfiguration(
                        "start_odometry_adapter"
                    ),
                }.items(),
                condition=IfCondition(
                    EqualsSubstitution(navigation_mode, "depth")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share, "launch", "b2arx_xt32_nav2.launch.py"
                    )
                ),
                launch_arguments={
                    "sensor_mode": "sim",
                    "use_sim_time": "true",
                    "start_hesai": "false",
                    "start_zed_wrapper": LaunchConfiguration(
                        "start_zed_wrapper"
                    ),
                    "start_odometry_adapter": LaunchConfiguration(
                        "start_odometry_adapter"
                    ),
                }.items(),
                condition=IfCondition(
                    EqualsSubstitution(navigation_mode, "lidar")
                ),
            ),
        ]
    )
