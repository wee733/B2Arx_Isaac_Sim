# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""XT32 PointCloud2 obstacle mode around the upstream Nav2 launch.

This mode deliberately does not start Nvblox. The ZED wrapper remains enabled
by default only because the current B2ARX platform adapter obtains localization
from ZED VIO. Disable both ``start_zed_wrapper`` and ``start_odometry_adapter``
when an external localization stack already provides ``/b2/odom`` and the
``map -> base_link`` TF chain.
"""

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
from launch_ros.actions import SetParameter


def _include_project_launch(
    bringup_share: str,
    filename: str,
    arguments: dict,
    *,
    condition=None,
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", filename)
        ),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    nav2_bt_share = get_package_share_directory("nav2_bt_navigator")

    sensor_mode = LaunchConfiguration("sensor_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    domain_id = LaunchConfiguration("domain_id")
    nav_params_file = LaunchConfiguration("nav_params_file")

    zed_localization = _include_project_launch(
        bringup_share,
        "zed.launch.py",
        {
            "sensor_mode": sensor_mode,
            "use_sim_time": use_sim_time,
            "sim_address": LaunchConfiguration("sim_address"),
            "sim_port": LaunchConfiguration("sim_port"),
            "serial_number": LaunchConfiguration("serial_number"),
            "camera_model": LaunchConfiguration("camera_model"),
            "start_zed_wrapper": LaunchConfiguration("start_zed_wrapper"),
            "disable_zed_nitros": LaunchConfiguration("disable_zed_nitros"),
            "zed_params_file": LaunchConfiguration("zed_params_file"),
            "zed_sdk_settings_path": LaunchConfiguration("zed_sdk_settings_path"),
        },
        condition=IfCondition(LaunchConfiguration("start_zed_wrapper")),
    )
    hesai = _include_project_launch(
        bringup_share,
        "hesai_xt32.launch.py",
        {
            "start_hesai": LaunchConfiguration("start_hesai"),
            "hesai_config_file": LaunchConfiguration("hesai_config_file"),
        },
    )
    nav2 = _include_project_launch(
        bringup_share,
        "nav2.launch.py",
        {
            "start_nav2": LaunchConfiguration("start_nav2"),
            "use_sim_time": use_sim_time,
            "autostart": LaunchConfiguration("autostart"),
            # No Nvblox component container exists in this mode.
            "use_composition": "false",
            "log_level": LaunchConfiguration("log_level"),
            "nav_params_file": nav_params_file,
            "nav_to_pose_bt_xml": LaunchConfiguration("nav_to_pose_bt_xml"),
        },
    )
    platform_adapters = _include_project_launch(
        bringup_share,
        "platform_adapters.launch.py",
        {
            "use_sim_time": use_sim_time,
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_odometry_adapter": LaunchConfiguration(
                "start_odometry_adapter"
            ),
            "publish_zed_mount_tf": LaunchConfiguration("publish_zed_mount_tf"),
            "publish_hesai_tf": LaunchConfiguration("publish_hesai_tf"),
            "platform_config_file": LaunchConfiguration("platform_config_file"),
        },
        condition=IfCondition(LaunchConfiguration("start_platform_adapters")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_mode", default_value="sim", choices=["sim", "real"]
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=PythonExpression(
                    ["'true' if '", sensor_mode, "' == 'sim' else 'false'"]
                ),
            ),
            DeclareLaunchArgument("domain_id", default_value="23"),
            DeclareLaunchArgument(
                "sim_address",
                default_value=EnvironmentVariable(
                    "SIM_ADDRESS", default_value="127.0.0.1"
                ),
            ),
            DeclareLaunchArgument(
                "sim_port",
                default_value=EnvironmentVariable("ZED_PORT", default_value="30000"),
            ),
            DeclareLaunchArgument("serial_number", default_value="0"),
            DeclareLaunchArgument("camera_model", default_value="zedx"),
            DeclareLaunchArgument(
                "start_zed_wrapper",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Start ZED only as the current localization/VIO provider; "
                    "its depth is not connected to either Nav2 costmap."
                ),
            ),
            DeclareLaunchArgument(
                "start_hesai",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Start the physical Hesai driver. Keep false in Isaac Sim, "
                    "which already publishes /lidar_points."
                ),
            ),
            DeclareLaunchArgument(
                "hesai_config_file",
                default_value=EnvironmentVariable(
                    "HESAI_CONFIG_FILE", default_value=""
                ),
            ),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_odometry_adapter", default_value="true"),
            DeclareLaunchArgument("start_platform_adapters", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("publish_zed_mount_tf", default_value="true"),
            DeclareLaunchArgument("publish_hesai_tf", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("disable_zed_nitros", default_value="true"),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "b2arx_nav2_xt32.yaml"
                ),
                description=(
                    "XT32-specific Nav2 parameters using ObstacleLayer on "
                    "/lidar_points."
                ),
            ),
            DeclareLaunchArgument(
                "zed_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "zedx_nvblox_release_4_5.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "platform_config_file",
                default_value=os.path.join(
                    bringup_share, "config", "platform_adapters.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "nav_to_pose_bt_xml",
                default_value=os.path.join(
                    nav2_bt_share,
                    "behavior_trees",
                    "navigate_w_replanning_time.xml",
                ),
            ),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value=EnvironmentVariable(
                    "RMW_IMPLEMENTATION", default_value="rmw_cyclonedds_cpp"
                ),
            ),
            DeclareLaunchArgument(
                "zed_sdk_settings_path",
                default_value=EnvironmentVariable(
                    "ZED_SDK_SETTINGS_PATH", default_value=""
                ),
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
            ),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("LC_ALL", "C"),
            GroupAction(
                actions=[
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    zed_localization,
                    hesai,
                    nav2,
                    platform_adapters,
                ]
            ),
        ]
    )
