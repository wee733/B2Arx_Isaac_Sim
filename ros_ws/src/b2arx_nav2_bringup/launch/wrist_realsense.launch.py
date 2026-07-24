# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Optional physical wrist D435i adapter around the official RealSense launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    realsense_share = get_package_share_directory("realsense2_camera")

    start_wrist_realsense = LaunchConfiguration("start_wrist_realsense")
    device_type = LaunchConfiguration("device_type")
    serial_no = LaunchConfiguration("serial_no")
    realsense_config_file = LaunchConfiguration("realsense_config_file")
    enable_imu = LaunchConfiguration("enable_imu")
    initial_reset = LaunchConfiguration("initial_reset")
    log_level = LaunchConfiguration("log_level")

    # Acquisition, timestamping, image alignment, IMU fusion, and the camera's
    # internal TF tree stay owned by the upstream RealSense driver. Keeping the
    # namespace empty and using wrist_camera as the node name intentionally
    # produces one prefix: /wrist_camera/..., not /wrist_camera/wrist_camera/....
    official_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        condition=IfCondition(start_wrist_realsense),
        launch_arguments={
            "camera_namespace": "",
            "camera_name": "wrist_camera",
            "serial_no": serial_no,
            "device_type": device_type,
            "config_file": realsense_config_file,
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "enable_gyro": enable_imu,
            "enable_accel": enable_imu,
            "unite_imu_method": "2",
            "publish_tf": "true",
            "initial_reset": initial_reset,
            "log_level": log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_wrist_realsense",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "device_type",
                default_value="d435i",
                description="RealSense device type selector passed to rs_launch.py.",
            ),
            DeclareLaunchArgument(
                "serial_no",
                default_value=EnvironmentVariable(
                    "WRIST_REALSENSE_SERIAL", default_value="''"
                ),
                description=(
                    "Physical D435i serial selector. For an all-numeric serial, "
                    "use the driver's string-safe _<serial> form."
                ),
            ),
            DeclareLaunchArgument(
                "realsense_config_file",
                default_value=os.path.join(
                    bringup_share, "config", "wrist_realsense_d435i.yaml"
                ),
                description="Flat parameter dictionary consumed by official rs_launch.py.",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                choices=["true", "false"],
                description="Enable both D435i gyro and accelerometer streams.",
            ),
            DeclareLaunchArgument(
                "initial_reset",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            official_realsense,
        ]
    )
