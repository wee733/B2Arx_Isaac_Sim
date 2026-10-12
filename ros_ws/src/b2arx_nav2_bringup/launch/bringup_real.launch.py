# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Real-robot profile selecting depth- or XT32-backed Nav2 costmaps."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from b2arx_nav2_bringup.launch_common import (
    BOOLEAN_CHOICES,
    HESAI_ARGUMENT_NAMES,
    forward,
    hesai_arguments,
    include_navigation_stack,
    include_project_launch,
    profile_arguments,
    project_file,
)


def generate_launch_description() -> LaunchDescription:
    wrist_realsense = include_project_launch(
        "wrist_realsense.launch.py",
        {
            "start_wrist_realsense": LaunchConfiguration("start_wrist_realsense"),
            "device_type": "d435i",
            "serial_no": LaunchConfiguration("wrist_realsense_serial"),
            "realsense_config_file": LaunchConfiguration(
                "wrist_realsense_config_file"
            ),
            "enable_imu": LaunchConfiguration("wrist_realsense_enable_imu"),
            "initial_reset": LaunchConfiguration("wrist_realsense_initial_reset"),
        },
    )

    return LaunchDescription(
        [
            *profile_arguments(),
            *hesai_arguments(),
            DeclareLaunchArgument(
                "start_wrist_realsense",
                default_value="false",
                choices=list(BOOLEAN_CHOICES),
            ),
            DeclareLaunchArgument(
                "wrist_realsense_serial",
                default_value=EnvironmentVariable(
                    "WRIST_REALSENSE_SERIAL", default_value="''"
                ),
            ),
            DeclareLaunchArgument(
                "wrist_realsense_config_file",
                default_value=project_file("config", "wrist_realsense_d435i.yaml"),
            ),
            DeclareLaunchArgument(
                "wrist_realsense_enable_imu",
                default_value="true",
                choices=list(BOOLEAN_CHOICES),
            ),
            DeclareLaunchArgument(
                "wrist_realsense_initial_reset",
                default_value="false",
                choices=list(BOOLEAN_CHOICES),
            ),
            # The navigation stack includes the Hesai adapter in both modes.
            include_navigation_stack("real", forward(*HESAI_ARGUMENT_NAMES)),
            wrist_realsense,
        ]
    )
