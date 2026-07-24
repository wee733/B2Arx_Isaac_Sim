# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Simulation profile for the shared Isaac ROS/Nav2 algorithm stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share, "launch", "isaac_ros_nav2.launch.py"
                    )
                ),
                launch_arguments={
                    "sensor_mode": "sim",
                    "use_sim_time": "true",
                }.items(),
            )
        ]
    )
