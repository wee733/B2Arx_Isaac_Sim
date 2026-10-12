# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Single B2ARX navigation stack entry, assembled from upstream launch files.

``navigation_mode`` selects the Nav2 obstacle source:

* ``depth`` (default): ZED depth -> Isaac ROS Nvblox -> NvbloxCostmapLayer.
  Nav2 is composed into the sole container created by the official Nvblox
  launch.
* ``lidar``: XT32 ``/lidar_points`` -> Nav2 ObstacleLayer. Nvblox is not
  started and Nav2 runs as standalone processes. The ZED wrapper stays enabled
  by default only because the current platform adapter obtains localization
  from ZED VIO; disable both ``start_zed_wrapper`` and
  ``start_odometry_adapter`` when an external localization stack already
  provides ``/b2/odom`` and the ``map -> base_link`` TF chain.

Both modes share one base Nav2 parameter file; lidar mode deep-merges the
costmap-only XT32 overlay over it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import SetParameter

from b2arx_nav2_bringup.launch_common import (
    HESAI_ARGUMENT_NAMES,
    NAV2_STANDALONE_CONTAINER_NAME,
    NAVIGATION_MODES,
    PLATFORM_ADAPTER_ARGUMENT_NAMES,
    XT32_NAV2_OVERLAY_FILE,
    ZED_ARGUMENT_NAMES,
    Nav2ParamsFile,
    config_equals,
    environment_actions,
    forward,
    hesai_arguments,
    include_project_launch,
    launch_config_bool,
    nav2_arguments,
    nvblox_container_name,
    platform_adapter_arguments,
    project_file,
    zed_arguments,
)


NAV2_REQUIRES_NVBLOX_CONTAINER_ERROR = (
    "navigation_mode:=depth composes Nav2 into the shared Nvblox container. "
    "start_nav2:=true therefore requires start_nvblox:=true. To run Nav2 "
    "without Nvblox, use navigation_mode:=lidar or launch nav2.launch.py "
    "directly; it defaults to use_composition:=false."
)


def _validate_shared_container_contract(context):
    if LaunchConfiguration("navigation_mode").perform(context) != "depth":
        return []
    if launch_config_bool(context, "start_nav2") and not launch_config_bool(
        context, "start_nvblox"
    ):
        raise RuntimeError(NAV2_REQUIRES_NVBLOX_CONTAINER_ERROR)
    return []


def generate_launch_description() -> LaunchDescription:
    depth_mode = config_equals("navigation_mode", "depth")

    def by_mode(depth_value, lidar_value):
        return IfElseSubstitution(depth_mode, if_value=depth_value, else_value=lidar_value)

    # The same inputs produce the same prepared file, so nav2.launch.py and
    # the shared Nvblox container consume one rewritten parameter file.
    nav_params = Nav2ParamsFile(
        LaunchConfiguration("nav_params_file"),
        LaunchConfiguration("nav_params_overlay_file"),
        LaunchConfiguration("nav_to_pose_bt_xml"),
    )

    zed = include_project_launch("zed.launch.py", forward(*ZED_ARGUMENT_NAMES))
    hesai = include_project_launch("hesai_xt32.launch.py", forward(*HESAI_ARGUMENT_NAMES))
    nvblox = include_project_launch(
        "nvblox.launch.py",
        {
            **forward(
                "start_nvblox",
                "use_sim_time",
                "container_name",
                "container_type",
                "log_level",
                "camera_model",
                "use_lidar_in_nvblox",
                "nvblox_params_file",
            ),
            "container_params_file": nav_params,
        },
        condition=IfCondition(depth_mode),
    )
    nav2 = include_project_launch(
        "nav2.launch.py",
        {
            **forward(
                "start_nav2",
                "use_sim_time",
                "autostart",
                "log_level",
                "nav_params_file",
                "nav_params_overlay_file",
                "nav_to_pose_bt_xml",
                "container_name",
            ),
            # Depth mode loads Nav2 into the Nvblox container; lidar mode has
            # no component container and runs the official Nav2 processes.
            "use_composition": by_mode("true", "false"),
        },
    )
    platform_adapters = include_project_launch(
        "platform_adapters.launch.py",
        forward("use_sim_time", *PLATFORM_ADAPTER_ARGUMENT_NAMES),
        condition=IfCondition(LaunchConfiguration("start_platform_adapters")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "navigation_mode",
                default_value="depth",
                choices=list(NAVIGATION_MODES),
                description=(
                    "depth: ZED + Isaac ROS Nvblox costmap; "
                    "lidar: XT32 PointCloud2 + Nav2 ObstacleLayer."
                ),
            ),
            *zed_arguments(),
            *hesai_arguments(),
            *platform_adapter_arguments(),
            DeclareLaunchArgument("start_platform_adapters", default_value="true"),
            *nav2_arguments(
                overlay_default=by_mode(
                    "", project_file("config", XT32_NAV2_OVERLAY_FILE)
                )
            ),
            DeclareLaunchArgument(
                "start_nvblox",
                default_value="true",
                description="Depth mode only; lidar mode never starts Nvblox.",
            ),
            DeclareLaunchArgument(
                "container_name",
                # Only depth mode resolves the upstream Nvblox constant.
                default_value=by_mode(
                    nvblox_container_name(), NAV2_STANDALONE_CONTAINER_NAME
                ),
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
            DeclareLaunchArgument(
                "nvblox_params_file",
                default_value=project_file("config", "nvblox_b2arx.yaml"),
            ),
            *environment_actions(),
            OpaqueFunction(function=_validate_shared_container_contract),
            GroupAction(
                actions=[
                    SetParameter(
                        name="use_sim_time", value=LaunchConfiguration("use_sim_time")
                    ),
                    zed,
                    hesai,
                    nvblox,
                    nav2,
                    platform_adapters,
                ]
            ),
        ]
    )
