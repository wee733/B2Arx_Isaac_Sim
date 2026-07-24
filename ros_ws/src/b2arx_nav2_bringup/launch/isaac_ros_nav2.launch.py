# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Composable B2ARX navigation stack assembled from upstream launch files."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
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
from nav2_common.launch import RewrittenYaml
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


SHARED_CONTAINER_NAME = NVBLOX_CONTAINER_NAME
NAV2_REQUIRES_NVBLOX_CONTAINER_ERROR = (
    "isaac_ros_nav2.launch.py composes Nav2 into the shared Nvblox container. "
    "start_nav2:=true therefore requires start_nvblox:=true. To run Nav2 "
    "without Nvblox, launch nav2.launch.py directly; it defaults to "
    "use_composition:=false."
)


def _launch_bool(context, name: str) -> bool:
    raw_value = LaunchConfiguration(name).perform(context)
    normalized = raw_value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a boolean value, got {raw_value!r}.")


def _validate_shared_container_contract(context):
    if _launch_bool(context, "start_nav2") and not _launch_bool(
        context, "start_nvblox"
    ):
        raise RuntimeError(NAV2_REQUIRES_NVBLOX_CONTAINER_ERROR)
    return []


def _include_project_launch(
    bringup_share: str,
    filename: str,
    arguments: dict,
    *,
    condition=None,
):
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
    log_level = LaunchConfiguration("log_level")
    container_type = LaunchConfiguration("container_type")
    container_name = LaunchConfiguration("container_name")
    nav_params = LaunchConfiguration("nav_params_file")
    nav_to_pose_bt_xml = LaunchConfiguration("nav_to_pose_bt_xml")

    rewritten_nav_params = RewrittenYaml(
        source_file=nav_params,
        param_rewrites={
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": (
                nav_to_pose_bt_xml
            ),
        },
        convert_types=True,
    )
    zed = _include_project_launch(
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
    )
    nvblox = _include_project_launch(
        bringup_share,
        "nvblox.launch.py",
        {
            "start_nvblox": LaunchConfiguration("start_nvblox"),
            "use_sim_time": use_sim_time,
            "container_name": container_name,
            "container_type": container_type,
            "container_params_file": rewritten_nav_params,
            "log_level": log_level,
            "camera_model": LaunchConfiguration("camera_model"),
            "use_lidar_in_nvblox": LaunchConfiguration("use_lidar_in_nvblox"),
            "nvblox_params_file": LaunchConfiguration("nvblox_params_file"),
        },
    )
    nav2 = _include_project_launch(
        bringup_share,
        "nav2.launch.py",
        {
            "start_nav2": LaunchConfiguration("start_nav2"),
            "use_sim_time": use_sim_time,
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": "true",
            "log_level": log_level,
            "container_name": container_name,
            "nav_params_file": nav_params,
            "nav_to_pose_bt_xml": nav_to_pose_bt_xml,
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
                description=(
                    "Isaac Sim host's reachable Ethernet address; a CLI value "
                    "overrides SIM_ADDRESS."
                ),
            ),
            DeclareLaunchArgument(
                "sim_port",
                default_value=EnvironmentVariable("ZED_PORT", default_value="30000"),
            ),
            DeclareLaunchArgument("serial_number", default_value="0"),
            DeclareLaunchArgument("camera_model", default_value="zedx"),
            DeclareLaunchArgument("start_zed_wrapper", default_value="true"),
            DeclareLaunchArgument("start_nvblox", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_odometry_adapter", default_value="true"),
            DeclareLaunchArgument("start_platform_adapters", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("publish_zed_mount_tf", default_value="true"),
            DeclareLaunchArgument("publish_hesai_tf", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "container_name", default_value=SHARED_CONTAINER_NAME
            ),
            DeclareLaunchArgument(
                "container_type",
                default_value="multithreaded",
                choices=["multithreaded", "isolated_multithreaded"],
                description=(
                    "The default lets the official Nvblox launch own the sole "
                    "component_container_mt. Isolated mode remains a project "
                    "compatibility fallback while keeping one container."
                ),
            ),
            DeclareLaunchArgument("use_lidar_in_nvblox", default_value="false"),
            DeclareLaunchArgument("disable_zed_nitros", default_value="true"),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "b2arx_nav2.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "zed_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "zedx_nvblox_release_4_5.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "nvblox_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "nvblox_b2arx.yaml"
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
            OpaqueFunction(function=_validate_shared_container_contract),
            GroupAction(
                actions=[
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    zed,
                    nvblox,
                    nav2,
                    platform_adapters,
                ]
            ),
        ]
    )
