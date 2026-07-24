# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""D435i eye-in-hand profile around the existing ARX Isaac ROS workflow."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


BEHAVIOR_CAMERA_FRAME_PATH = (
    "behavior_tree_params.multi_object_pick_and_place."
    "pose_estimation.camera_frame_id"
)


def generate_launch_description() -> LaunchDescription:
    adapter_share = get_package_share_directory("b2arx_nav2_bringup")
    manipulation_share = get_package_share_directory(
        "isaac_ros_manipulation_arx_r5a_bringup"
    )

    behavior_tree_source_file = LaunchConfiguration("behavior_tree_source_file")
    # GetObjectPose has no Header. A wrist camera therefore must transform the
    # detection at image time and hand the behavior tree a base_link pose.
    wrist_behavior_tree = RewrittenYaml(
        source_file=behavior_tree_source_file,
        param_rewrites={BEHAVIOR_CAMERA_FRAME_PATH: "base_link"},
        convert_types=True,
    )

    upstream_workflow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                manipulation_share,
                "launch",
                "arx_r5a_apriltag_pick_and_place.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "False",
            "headless": LaunchConfiguration("headless"),
            "log_level": LaunchConfiguration("log_level"),
            "start_apriltag": LaunchConfiguration("start_apriltag"),
            "start_object_selection_server": LaunchConfiguration(
                "start_object_selection_server"
            ),
            "start_robot": LaunchConfiguration("start_robot"),
            "start_motion_stack": LaunchConfiguration("start_motion_stack"),
            "start_vendor_driver": "False",
            "start_rviz": LaunchConfiguration("start_rviz"),
            "start_orchestrator": LaunchConfiguration("start_orchestrator"),
            "enable_nvblox": LaunchConfiguration("enable_nvblox"),
            "camera_width": "640",
            "camera_height": "480",
            "perception_namespace": "wrist_camera",
            "color_image_topic": LaunchConfiguration("color_image_topic"),
            "color_camera_info_topic": LaunchConfiguration(
                "color_camera_info_topic"
            ),
            "depth_image_topic": LaunchConfiguration("depth_image_topic"),
            "depth_camera_info_topic": LaunchConfiguration(
                "depth_camera_info_topic"
            ),
            "robot_mask_topic": "/cumotion/wrist_camera/robot_mask",
            "world_depth_topic": "/cumotion/wrist_camera/world_depth",
            "tag_detections_topic": "/wrist_camera/tag_detections",
            "camera_optical_frame": "wrist_camera_color_optical_frame",
            "output_frame": "base_link",
            "camera_calibration_file": LaunchConfiguration(
                "camera_calibration_file"
            ),
            "behavior_tree_config_file": wrist_behavior_tree,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="False"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("start_apriltag", default_value="True"),
            DeclareLaunchArgument(
                "start_object_selection_server", default_value="True"
            ),
            DeclareLaunchArgument("start_robot", default_value="True"),
            DeclareLaunchArgument("start_motion_stack", default_value="True"),
            DeclareLaunchArgument("start_rviz", default_value="True"),
            DeclareLaunchArgument("start_orchestrator", default_value="True"),
            DeclareLaunchArgument("enable_nvblox", default_value="True"),
            DeclareLaunchArgument(
                "color_image_topic",
                default_value="/wrist_camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "color_camera_info_topic",
                default_value="/wrist_camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/wrist_camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/wrist_camera/aligned_depth_to_color/camera_info",
            ),
            DeclareLaunchArgument(
                "camera_calibration_file",
                default_value=os.path.join(
                    adapter_share, "config", "wrist_d435i_eye_in_hand.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "behavior_tree_source_file",
                default_value=os.path.join(
                    manipulation_share,
                    "config",
                    "behavior_tree",
                    "arx_r5a_apriltag_behavior_tree.yaml",
                ),
                description=(
                    "Existing ARX behavior-tree profile; only "
                    "pose_estimation.camera_frame_id is rewritten to base_link."
                ),
            ),
            upstream_workflow,
        ]
    )
