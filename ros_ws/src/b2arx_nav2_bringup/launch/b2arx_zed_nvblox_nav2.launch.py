# Copyright 2026 lbz
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode


# Robot-specific seam between the official ZED X xacro and the simulated B2 root.
#
# B2 base -> R5a            = (0.220, 0, 0.100)
# R5a -> official USD mount = (0.280, 0, -0.030)
# Both Stereolabs definitions use the bottom mounting point: USD base_link and
# wrapper zed_camera_link. Therefore the inverse installation edge is below.
ZED_CAMERA_LINK_TO_B2_BASE = (-0.500, 0.0, -0.070)


def _nav2_nodes(params_file, log_level, nav_to_pose_bt_xml):
    common = {
        "output": "screen",
        "parameters": [params_file],
        "arguments": ["--ros-args", "--log-level", log_level],
    }
    nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=[("cmd_vel", "cmd_vel_nav")],
            **common,
        ),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", **common),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", **common),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            remappings=[("cmd_vel", "cmd_vel_nav")],
            **common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            # model_29999 has no pure in-place rotation contract. Use Nav2's
            # official periodic-replanning tree without Spin recovery. This
            # launch-level override keeps the YAML portable because the XML
            # path is resolved from nav2_bt_navigator's installed share dir.
            parameters=[
                params_file,
                {"default_nav_to_pose_bt_xml": nav_to_pose_bt_xml},
            ],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            **common,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel_smoothed"),
            ],
            **common,
        ),
    ]
    lifecycle_names = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
        "waypoint_follower",
    ]
    # Fast DDS can discover a lifecycle request endpoint before its response
    # endpoint when many processes start together.  The transition then
    # succeeds server-side, but the manager waits forever for a response that
    # could not be sent (ros2/rmw_fastrtps#842).  Let the Nav2 service graph
    # settle after the ZED/Nvblox cold start before beginning autostart.
    nodes.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[
                        {"use_sim_time": True},
                        {"autostart": True},
                        {"node_names": lifecycle_names},
                    ],
                )
            ],
        )
    )
    return nodes


def generate_launch_description():
    bringup_share = get_package_share_directory("b2arx_nav2_bringup")
    nav2_bt_navigator_share = get_package_share_directory("nav2_bt_navigator")
    zed_share = get_package_share_directory("zed_wrapper")

    domain_id = LaunchConfiguration("domain_id")
    sim_address = LaunchConfiguration("sim_address")
    sim_port = LaunchConfiguration("sim_port")
    zed_sdk_settings_path = LaunchConfiguration("zed_sdk_settings_path")
    start_zed_wrapper = LaunchConfiguration("start_zed_wrapper")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    nav_params = f"{bringup_share}/config/b2arx_nav2.yaml"
    zed_params = f"{bringup_share}/config/zedx_nvblox_release_4_5.yaml"
    nvblox_base_params = f"{bringup_share}/config/nvblox_base_release_4_5.yaml"
    nvblox_zed_params = f"{bringup_share}/config/nvblox_zed_release_4_5.yaml"
    nav_to_pose_bt_xml = (
        f"{nav2_bt_navigator_share}/behavior_trees/navigate_w_replanning_time.xml"
    )

    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{zed_share}/launch/zed_camera.launch.py"),
        condition=IfCondition(start_zed_wrapper),
        launch_arguments={
            "camera_model": "zedx",
            "sim_mode": "true",
            "use_sim_time": "true",
            "sim_address": sim_address,
            "sim_port": sim_port,
            "publish_tf": "true",
            "publish_map_tf": "true",
            "ros_params_override_path": zed_params,
        }.items(),
    )

    camera_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_camera_to_b2_base",
        output="screen",
        arguments=[
            "--x", str(ZED_CAMERA_LINK_TO_B2_BASE[0]),
            "--y", str(ZED_CAMERA_LINK_TO_B2_BASE[1]),
            "--z", str(ZED_CAMERA_LINK_TO_B2_BASE[2]),
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "zed_camera_link",
            "--child-frame-id", "base_link",
        ],
        parameters=[{"use_sim_time": True}],
    )

    nvblox_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="nvblox_container",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=[
            ComposableNode(
                package="nvblox_ros",
                plugin="nvblox::NvbloxNode",
                name="nvblox_node",
                parameters=[
                    nvblox_base_params,
                    nvblox_zed_params,
                    {
                        "num_cameras": 1,
                        "use_lidar": False,
                        "use_sim_time": True,
                        # The ZED pose input is expressed in map. Keep Nvblox
                        # and both Nav2 costmaps in that frame so the strictly
                        # planar costmap layer never projects map->odom.
                        "global_frame": "map",
                    },
                ],
                remappings=[
                    ("camera_0/depth/image", "/zed/zed_node/depth/depth_registered"),
                    ("camera_0/depth/camera_info", "/zed/zed_node/depth/camera_info"),
                    ("camera_0/color/image", "/zed/zed_node/rgb/color/rect/image"),
                    (
                        "camera_0/color/camera_info",
                        "/zed/zed_node/rgb/color/rect/camera_info",
                    ),
                    ("pose", "/zed/zed_node/pose"),
                ],
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", f"{bringup_share}/rviz/b2arx_nvblox_nav2.rviz"],
        parameters=[{"use_sim_time": True}],
    )

    # This regular (non-lifecycle) transport adapter stays outside the scoped
    # use_sim_time group.  Its 0.5 s fail-zero timeout uses a steady wall clock,
    # while all navigation/perception nodes continue to use simulation time.
    cmd_vel_watchdog = Node(
        package="b2arx_nav2_bringup",
        executable="cmd_vel_watchdog",
        name="cmd_vel_watchdog",
        output="screen",
        parameters=[nav_params],
        remappings=[
            ("cmd_vel_in", "cmd_vel_smoothed"),
            ("cmd_vel_out", "cmd_vel"),
            ("heartbeat_out", "cmd_vel_heartbeat"),
        ],
    )

    # Keep the behavior server's official CostmapTopicCollisionChecker, but
    # provide its static footprint directly in base_link. This avoids asking
    # the behavior process to query the costmap process's latest exact TF stamp.
    behavior_footprint = Node(
        package="b2arx_nav2_bringup",
        executable="behavior_footprint_publisher",
        name="behavior_footprint_publisher",
        output="screen",
        parameters=[nav_params],
        remappings=[("footprint_out", "behavior_footprint")],
    )

    runtime = GroupAction(
        actions=[
            SetParameter(name="use_sim_time", value=True),
            zed_wrapper,
            camera_to_base,
            nvblox_container,
            *_nav2_nodes(nav_params, log_level, nav_to_pose_bt_xml),
            rviz,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("domain_id", default_value="23"),
        DeclareLaunchArgument("sim_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("sim_port", default_value="30000"),
        DeclareLaunchArgument(
            "zed_sdk_settings_path",
            default_value=EnvironmentVariable("ZED_SDK_SETTINGS_PATH", default_value=""),
        ),
        DeclareLaunchArgument("start_zed_wrapper", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", domain_id),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        # ZED SDK explicitly requires the C numeric locale when parsing its
        # factory-style SN calibration file.
        SetEnvironmentVariable("LC_ALL", "C"),
        SetEnvironmentVariable("ZED_SDK_SETTINGS_PATH", zed_sdk_settings_path),
        behavior_footprint,
        cmd_vel_watchdog,
        runtime,
    ])
