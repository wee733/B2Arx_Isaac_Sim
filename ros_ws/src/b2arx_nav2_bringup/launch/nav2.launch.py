# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Nav2 official bringup plus the B2ARX command-safety adapters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


UNSUPPORTED_DOCKING_CMD_VEL_TOPIC = "/cmd_vel_docking_unsupported"
DEFAULT_NAV2_CONTAINER_NAME = "nav2_container"


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_bt_share = get_package_share_directory("nav2_bt_navigator")

    start_nav2 = LaunchConfiguration("start_nav2")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    nav_params = LaunchConfiguration("nav_params_file")
    nav_to_pose_bt_xml = LaunchConfiguration("nav_to_pose_bt_xml")
    container_name = LaunchConfiguration("container_name")
    use_composition = LaunchConfiguration("use_composition")
    log_level = LaunchConfiguration("log_level")
    official_use_composition = IfElseSubstitution(
        use_composition,
        if_value="True",
        else_value="False",
    )

    rewritten_nav_params = RewrittenYaml(
        source_file=nav_params,
        param_rewrites={
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": (
                nav_to_pose_bt_xml
            ),
        },
        convert_types=True,
    )
    configured_nav_params = ParameterFile(rewritten_nav_params, allow_substs=True)

    official_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": rewritten_nav_params,
            # navigation_launch.py uses PythonExpression internally, so pass
            # Python boolean spelling after accepting normal ROS lowercase
            # booleans at this wrapper's CLI.
            "use_composition": official_use_composition,
            "container_name": container_name,
            "use_respawn": "false",
            "log_level": log_level,
        }.items(),
    )

    cmd_vel_watchdog = Node(
        package="b2arx_nav2_bringup",
        executable="cmd_vel_watchdog",
        name="cmd_vel_watchdog",
        output="screen",
        parameters=[configured_nav_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel_in", "cmd_vel_smoothed"),
            ("cmd_vel_out", "cmd_vel"),
            ("heartbeat_out", "cmd_vel_heartbeat"),
        ],
    )
    behavior_footprint = Node(
        package="b2arx_nav2_bringup",
        executable="behavior_footprint_publisher",
        name="behavior_footprint_publisher",
        output="screen",
        parameters=[configured_nav_params, {"use_sim_time": use_sim_time}],
        remappings=[("footprint_out", "behavior_footprint")],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "use_composition",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Standalone module calls default to normal Nav2 processes. "
                    "Set true only when container_name already exists."
                ),
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_NAV2_CONTAINER_NAME
            ),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=os.path.join(
                    bringup_share, "config", "b2arx_nav2.yaml"
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
            GroupAction(
                condition=IfCondition(start_nav2),
                actions=[
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    # Jazzy's docking server publishes cmd_vel directly. Keep
                    # the official server loaded while isolating that command
                    # path until a B2 policy-aware docking adapter is provided.
                    SetRemap(
                        src="docking_server:cmd_vel",
                        dst=UNSUPPORTED_DOCKING_CMD_VEL_TOPIC,
                    ),
                    official_navigation,
                    cmd_vel_watchdog,
                    behavior_footprint,
                ],
            ),
        ]
    )
