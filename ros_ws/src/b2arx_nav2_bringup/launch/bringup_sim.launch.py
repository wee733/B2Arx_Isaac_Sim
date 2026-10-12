# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Simulation profile selecting depth- or XT32-backed Nav2 costmaps."""

from launch import LaunchDescription

from b2arx_nav2_bringup.launch_common import (
    include_navigation_stack,
    profile_arguments,
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            *profile_arguments(),
            # Isaac Sim already publishes /lidar_points; never start the
            # physical Hesai driver in this profile.
            include_navigation_stack("sim", {"start_hesai": "false"}),
        ]
    )
