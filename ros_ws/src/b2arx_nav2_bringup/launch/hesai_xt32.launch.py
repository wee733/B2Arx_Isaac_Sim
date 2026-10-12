# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Optional physical XT32 adapter around the official Hesai driver node."""

import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from b2arx_nav2_bringup.launch_common import hesai_arguments, launch_config_bool


def _validate_hesai_config(context):
    if not launch_config_bool(context, "start_hesai"):
        return []

    config_path = LaunchConfiguration("hesai_config_file").perform(context).strip()
    if not config_path:
        raise RuntimeError(
            "start_hesai:=true requires hesai_config_file:=/absolute/config.yaml "
            "or the HESAI_CONFIG_FILE environment variable."
        )
    if not os.path.isfile(config_path):
        raise RuntimeError(f"Hesai config file does not exist: {config_path}")
    return []


def generate_launch_description() -> LaunchDescription:
    start_hesai = LaunchConfiguration("start_hesai")
    hesai_config_file = LaunchConfiguration("hesai_config_file")

    # The vendor launch forces RViz and does not expose config_path. Reuse the
    # vendor executable directly and keep all packet parsing/calibration in the
    # official hesai_ros_driver package.
    hesai_driver = Node(
        condition=IfCondition(start_hesai),
        namespace="hesai_ros_driver",
        package="hesai_ros_driver",
        executable="hesai_ros_driver_node",
        name="hesai_ros_driver_node",
        output="screen",
        parameters=[{"config_path": hesai_config_file}],
    )

    return LaunchDescription(
        [
            *hesai_arguments(),
            OpaqueFunction(function=_validate_hesai_config),
            hesai_driver,
        ]
    )
