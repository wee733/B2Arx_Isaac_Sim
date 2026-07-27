# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Real-robot profile selecting depth- or XT32-backed Nav2 costmaps."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    EqualsSubstitution,
    LaunchConfiguration,
)


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    navigation_mode = LaunchConfiguration("navigation_mode")
    project_launch = lambda name: PythonLaunchDescriptionSource(  # noqa: E731
        os.path.join(bringup_share, "launch", name)
    )

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
            DeclareLaunchArgument("start_hesai", default_value="false"),
            DeclareLaunchArgument(
                "hesai_config_file",
                default_value=EnvironmentVariable(
                    "HESAI_CONFIG_FILE", default_value=""
                ),
            ),
            DeclareLaunchArgument(
                "start_wrist_realsense",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "wrist_realsense_serial",
                default_value=EnvironmentVariable(
                    "WRIST_REALSENSE_SERIAL", default_value="''"
                ),
            ),
            DeclareLaunchArgument(
                "wrist_realsense_config_file",
                default_value=os.path.join(
                    bringup_share, "config", "wrist_realsense_d435i.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "wrist_realsense_enable_imu",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "wrist_realsense_initial_reset",
                default_value="false",
                choices=["true", "false"],
            ),
            IncludeLaunchDescription(
                project_launch("isaac_ros_nav2.launch.py"),
                launch_arguments={
                    "sensor_mode": "real",
                    "use_sim_time": "false",
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
                project_launch("b2arx_xt32_nav2.launch.py"),
                launch_arguments={
                    "sensor_mode": "real",
                    "use_sim_time": "false",
                    "start_hesai": LaunchConfiguration("start_hesai"),
                    "hesai_config_file": LaunchConfiguration(
                        "hesai_config_file"
                    ),
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
            IncludeLaunchDescription(
                project_launch("hesai_xt32.launch.py"),
                launch_arguments={
                    "start_hesai": LaunchConfiguration("start_hesai"),
                    "hesai_config_file": LaunchConfiguration("hesai_config_file"),
                }.items(),
                condition=IfCondition(
                    EqualsSubstitution(navigation_mode, "depth")
                ),
            ),
            IncludeLaunchDescription(
                project_launch("wrist_realsense.launch.py"),
                launch_arguments={
                    "start_wrist_realsense": LaunchConfiguration(
                        "start_wrist_realsense"
                    ),
                    "device_type": "d435i",
                    "serial_no": LaunchConfiguration("wrist_realsense_serial"),
                    "realsense_config_file": LaunchConfiguration(
                        "wrist_realsense_config_file"
                    ),
                    "enable_imu": LaunchConfiguration(
                        "wrist_realsense_enable_imu"
                    ),
                    "initial_reset": LaunchConfiguration(
                        "wrist_realsense_initial_reset"
                    ),
                }.items(),
            ),
        ]
    )
