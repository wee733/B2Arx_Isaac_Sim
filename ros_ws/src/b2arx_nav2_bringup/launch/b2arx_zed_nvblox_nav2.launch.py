# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Compatibility alias for the original B2ARX simulation launch command."""

from launch import LaunchDescription

from b2arx_nav2_bringup.launch_common import include_project_launch


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([include_project_launch("bringup_sim.launch.py")])
