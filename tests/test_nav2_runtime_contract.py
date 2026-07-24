from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / "ros_ws" / "src" / "b2arx_nav2_bringup"


def _launch_text(filename: str) -> str:
    return (BRINGUP / "launch" / filename).read_text(encoding="utf-8")


def test_nav2_composition_targets_the_official_nvblox_container() -> None:
    main = _launch_text("isaac_ros_nav2.launch.py")
    zed = _launch_text("zed.launch.py")
    nvblox = _launch_text("nvblox.launch.py")
    nav2 = _launch_text("nav2.launch.py")
    legacy = _launch_text("b2arx_zed_nvblox_nav2.launch.py")

    assert "SHARED_CONTAINER_NAME = NVBLOX_CONTAINER_NAME" in main
    assert 'default_value="multithreaded"' in main
    assert 'choices=["multithreaded", "isolated_multithreaded"]' in main
    assert 'package="rclcpp_components"' not in main
    assert 'executable="component_container_mt"' not in main
    assert 'executable="component_container_isolated"' not in main
    assert '"container_params_file": rewritten_nav_params' in main

    # Algorithms stay in their upstream packages; the project only includes
    # the official launch APIs and applies configuration/remapping seams.
    assert '"launch", "zed_camera.launch.py"' in zed
    assert '"container_name": ""' in zed
    assert '"enable_ipc": "false"' in zed
    assert '"debug.disable_nitros:="' in zed
    assert '"perception",\n                "nvblox.launch.py"' in nvblox
    assert '"run_standalone": IfElseSubstitution(' in nvblox
    assert 'if_value="True"' in nvblox
    assert 'else_value="False"' in nvblox
    assert '"run_standalone": "false"' not in nvblox
    assert "SetParametersFromFile(container_params_file)" in nvblox
    assert "SetParametersFromFile(nvblox_params_file)" in nvblox
    assert 'executable="component_container_isolated"' in nvblox
    assert nvblox.count('executable="component_container_isolated"') == 1
    assert '"--use_multi_threaded_executor"' in nvblox
    assert '"launch", "navigation_launch.py"' in nav2
    assert 'official_use_composition = IfElseSubstitution(' in nav2
    assert 'if_value="True"' in nav2
    assert 'else_value="False"' in nav2
    assert '"use_composition": official_use_composition' in nav2
    assert 'LaunchConfiguration("use_composition")' in nav2
    assert '"use_composition",\n                default_value="false"' in nav2
    assert 'choices=["true", "false"]' in nav2
    assert '"use_composition": "true"' in main
    assert '"container_name": container_name' in nav2
    assert "ComposableNode(" not in zed + nvblox + nav2
    assert '"bringup_sim.launch.py"' in legacy


def test_top_level_composition_requires_the_nvblox_container_owner() -> None:
    main = _launch_text("isaac_ros_nav2.launch.py")

    assert "NAV2_REQUIRES_NVBLOX_CONTAINER_ERROR" in main
    assert "def _validate_shared_container_contract(context):" in main
    assert '_launch_bool(context, "start_nav2")' in main
    assert 'context, "start_nvblox"' in main
    assert "raise RuntimeError(NAV2_REQUIRES_NVBLOX_CONTAINER_ERROR)" in main
    assert "OpaqueFunction(function=_validate_shared_container_contract)" in main


def test_nav2_velocity_path_is_smoother_watchdog_policy() -> None:
    params = yaml.safe_load(
        (BRINGUP / "config" / "b2arx_nav2.yaml").read_text(encoding="utf-8")
    )
    launch_text = _launch_text("nav2.launch.py")
    platform_text = _launch_text("platform_adapters.launch.py")

    collision_monitor = params["collision_monitor"]["ros__parameters"]
    assert collision_monitor["enabled"] is False
    assert collision_monitor["cmd_vel_in_topic"] == "cmd_vel_smoothed"
    assert collision_monitor["cmd_vel_out_topic"] == "cmd_vel_collision_monitor"
    assert '("cmd_vel_in", "cmd_vel_smoothed")' in launch_text
    assert '("cmd_vel_out", "cmd_vel")' in launch_text
    assert '("heartbeat_out", "cmd_vel_heartbeat")' in launch_text
    assert 'executable="odometry_adapter"' not in launch_text
    assert 'executable="odometry_adapter"' in platform_text
    assert 'DeclareLaunchArgument("start_odometry_adapter", default_value="true")' in platform_text
    assert 'LaunchConfiguration("platform_config_file")' in platform_text
    assert 'src="docking_server:cmd_vel"' in launch_text
    assert (
        'UNSUPPORTED_DOCKING_CMD_VEL_TOPIC = "/cmd_vel_docking_unsupported"'
        in launch_text
    )


def test_nav2_exposes_only_the_no_spin_position_goal_contract() -> None:
    params = yaml.safe_load(
        (BRINGUP / "config" / "b2arx_nav2.yaml").read_text(encoding="utf-8")
    )
    launch_text = _launch_text("nav2.launch.py")

    bt_navigator = params["bt_navigator"]["ros__parameters"]
    controller = params["controller_server"]["ros__parameters"]
    planner = params["planner_server"]["ros__parameters"]["GridBased"]

    assert bt_navigator["navigators"] == ["navigate_to_pose"]
    assert "navigate_through_poses" not in bt_navigator
    assert "navigate_w_replanning_time.xml" in launch_text
    assert controller["goal_checker_plugins"] == ["position_goal_checker"]
    assert controller["position_goal_checker"]["plugin"] == (
        "nav2_controller::PositionGoalChecker"
    )
    assert controller["FollowPath"]["use_rotate_to_heading"] is False
    assert planner["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert planner["use_final_approach_orientation"] is True


def test_nav2_bringup_declares_every_extra_jazzy_navigation_component() -> None:
    package_xml = (BRINGUP / "package.xml").read_text(encoding="utf-8")
    setup_py = (BRINGUP / "setup.py").read_text(encoding="utf-8")

    for dependency in (
        "nav2_collision_monitor",
        "nav2_route",
        "nav_msgs",
        "opennav_docking",
        "python3-yaml",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_xml
    assert "odometry_adapter = b2arx_nav2_bringup.odometry_adapter:main" in setup_py


def test_nvblox_costmap_layer_topic_and_frame_match_the_map_slice() -> None:
    params = yaml.safe_load(
        (BRINGUP / "config" / "b2arx_nav2.yaml").read_text(encoding="utf-8")
    )

    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = params[costmap_name][costmap_name]["ros__parameters"]
        layer = costmap["nvblox_layer"]
        assert costmap["global_frame"] == "map"
        assert layer["nav2_costmap_global_frame"] == costmap["global_frame"]
        assert layer["nvblox_map_slice_topic"] == "/nvblox_node/static_map_slice"
        assert layer["plugin"] == "nvblox::nav2::NvbloxCostmapLayer"


def test_nvblox_defaults_to_zed_only_and_rejects_lidar_fusion() -> None:
    nvblox = yaml.safe_load(
        (BRINGUP / "config" / "nvblox_b2arx.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    launch_text = _launch_text("nvblox.launch.py")
    platform_config = yaml.safe_load(
        (BRINGUP / "config" / "platform_adapters.yaml").read_text(encoding="utf-8")
    )

    # Keep the official ZED depth/color path. The XT32 is an independent ROS
    # PointCloud2 source and must not be fused into this Nvblox instance.
    assert nvblox["num_cameras"] == 1
    assert nvblox["use_depth"] is True
    assert '"camera": zed_camera_model' in launch_text
    assert '"mode": "static"' in launch_text
    assert '"num_cameras": "1"' in launch_text
    assert '"run_standalone": IfElseSubstitution(' in launch_text
    assert 'if_value="True"' in launch_text
    assert 'else_value="False"' in launch_text
    assert "SetParametersFromFile(container_params_file)" in launch_text
    assert 'DeclareLaunchArgument(\n                "container_params_file"' in launch_text
    assert "camera_0/depth/image" not in launch_text
    assert "camera_0/color/image" not in launch_text
    assert "pointcloud" not in launch_text
    hesai_mount = platform_config["mounts"]["base_to_hesai_lidar"]
    assert hesai_mount["child_frame"] == "hesai_lidar"

    assert nvblox["use_lidar"] is False
    assert 'LaunchConfiguration("use_lidar_in_nvblox")' in launch_text
    assert '"use_lidar_in_nvblox",\n                default_value="false"' in launch_text
    assert "OpaqueFunction(function=_validate_sensor_contract)" in launch_text
    assert "raise RuntimeError(NVBLOX_ZED_LIDAR_ERROR)" in launch_text
    assert "Isaac ROS 4.5 ZED" in launch_text
    assert '"lidar": use_lidar' in launch_text


def test_sim_and_real_profiles_share_algorithms_and_swap_sensor_adapters() -> None:
    sim = _launch_text("bringup_sim.launch.py")
    real = _launch_text("bringup_real.launch.py")
    hesai = _launch_text("hesai_xt32.launch.py")

    for profile in (sim, real):
        assert '"isaac_ros_nav2.launch.py"' in profile
    assert '"sensor_mode": "sim"' in sim
    assert '"use_sim_time": "true"' in sim
    assert '"sensor_mode": "real"' in real
    assert '"use_sim_time": "false"' in real
    assert '"hesai_xt32.launch.py"' in real

    assert 'package="hesai_ros_driver"' in hesai
    assert 'executable="hesai_ros_driver_node"' in hesai
    assert 'parameters=[{"config_path": hesai_config_file}]' in hesai
    assert '"HESAI_CONFIG_FILE", default_value=""' in hesai
    assert 'DeclareLaunchArgument("start_hesai", default_value="false")' in hesai
    assert "send_point_cloud_ros" not in hesai


def test_real_wrist_d435i_reuses_the_official_realsense_launch() -> None:
    real = _launch_text("bringup_real.launch.py")
    wrist = _launch_text("wrist_realsense.launch.py")
    package_xml = (BRINGUP / "package.xml").read_text(encoding="utf-8")
    config = yaml.safe_load(
        (BRINGUP / "config" / "wrist_realsense_d435i.yaml").read_text(
            encoding="utf-8"
        )
    )

    # The project is only an adapter: device discovery, frames, streams, and
    # hardware timestamping remain implemented by the installed vendor driver.
    assert 'get_package_share_directory("realsense2_camera")' in wrist
    assert '"launch", "rs_launch.py"' in wrist
    assert 'executable="realsense2_camera_node"' not in wrist
    assert '"camera_namespace": ""' in wrist
    assert '"camera_name": "wrist_camera"' in wrist
    assert 'default_value="d435i"' in wrist
    assert '"WRIST_REALSENSE_SERIAL", default_value="\'\'"' in wrist
    assert '"enable_color": "true"' in wrist
    assert '"enable_depth": "true"' in wrist
    assert '"align_depth.enable": "true"' in wrist
    assert '"enable_sync": "true"' in wrist
    assert '"enable_gyro": enable_imu' in wrist
    assert '"enable_accel": enable_imu' in wrist
    assert '"unite_imu_method": "2"' in wrist
    assert '"publish_tf": "true"' in wrist

    assert '"wrist_realsense.launch.py"' in real
    assert '"start_wrist_realsense": LaunchConfiguration(' in real
    assert '"serial_no": LaunchConfiguration("wrist_realsense_serial")' in real
    assert '"realsense_config_file": LaunchConfiguration(' in real
    assert '"enable_imu": LaunchConfiguration(' in real
    assert '<exec_depend>realsense2_camera</exec_depend>' in package_xml

    # rs_launch.py calls yaml.safe_load() and passes this flat mapping directly
    # to its node. IMU stays out so enable_imu:=false remains authoritative.
    assert config == {
        "enable_color": True,
        "rgb_camera.color_profile": "640x480x30",
        "enable_depth": True,
        "depth_module.depth_profile": "640x480x30",
        "align_depth.enable": True,
        "enable_sync": True,
    }


def test_rviz_displays_the_xt32_pointcloud_with_axis_color() -> None:
    rviz = yaml.safe_load(
        (BRINGUP / "rviz" / "b2arx_nvblox_nav2.rviz").read_text(encoding="utf-8")
    )
    displays = rviz["Visualization Manager"]["Displays"]
    xt32 = next(display for display in displays if display.get("Name") == "XT32 Point Cloud")

    assert xt32["Class"] == "rviz_default_plugins/PointCloud2"
    assert xt32["Enabled"] is True
    assert xt32["Position Transformer"] == "XYZ"
    assert xt32["Color Transformer"] == "AxisColor"
    assert xt32["Axis"] == "Z"
    assert xt32["Topic"]["Value"] == "/lidar_points"
    assert xt32["Topic"]["Reliability Policy"] == "Best Effort"


def test_runtime_check_validates_the_xt32_pointcloud_and_tf_contract() -> None:
    script = (ROOT / "scripts" / "check_isaac_ros_runtime.sh").read_text(
        encoding="utf-8"
    )

    assert "ros2 topic info /lidar_points --verbose" in script
    assert "Type: sensor_msgs/msg/PointCloud2" in script
    assert "Publisher\\ count:\\ [1-9][0-9]*" in script
    assert "require_nvblox_lidar" not in script
    assert "--require-nvblox-lidar" not in script
    assert "ros2 param get /nvblox_node use_lidar" in script
    assert '[[ "$nvblox_lidar_parameter" == *False* ]]' in script
    assert "requires use_lidar=false" in script
    assert "independent of ZED Nvblox" in script
    assert "ros2 topic echo /lidar_points sensor_msgs/msg/PointCloud2" in script
    assert "--field header.frame_id --once" in script
    assert '"$lidar_frame_id" == *hesai_lidar*' in script
    assert "--field fields --once" in script
    assert "for field_name in x y z" in script
    assert 'grep -Eq "name[=:][[:space:]]*' in script
    assert "ros2 run tf2_ros tf2_echo base_link hesai_lidar" in script
    assert "timeout 15s" in script

    # XT32 is deliberately an independent PointCloud2 source. Its checker
    # must verify the publisher and payload contract without requiring any
    # subscriber, especially nvblox_node.
    lidar_endpoint_check = script.split('lidar_info=""', 1)[1].split(
        'lidar_frame_id="$(',
        1,
    )[0]
    assert "Subscription\\ count" not in lidar_endpoint_check
    assert "Node name: nvblox_node" not in lidar_endpoint_check


def test_runtime_check_can_require_the_normalized_wrist_rgbd_contract() -> None:
    script = (ROOT / "scripts" / "check_isaac_ros_runtime.sh").read_text(
        encoding="utf-8"
    )

    assert "--require-wrist-camera" in script
    for topic in (
        "/wrist_camera/color/image_raw",
        "/wrist_camera/color/camera_info",
        "/wrist_camera/aligned_depth_to_color/image_raw",
        "/wrist_camera/aligned_depth_to_color/camera_info",
    ):
        assert topic in script
    assert "wrist_camera_color_optical_frame" in script
    assert "32FC1 metres (Isaac Sim)" in script
    assert "16UC1 millimetres / depth_scale 0.001 (RealSense)" in script
    assert "unsupported wrist aligned-depth encoding" in script


def test_runtime_check_requires_live_zed_publishers_messages_and_tf() -> None:
    script = (ROOT / "scripts" / "check_isaac_ros_runtime.sh").read_text(
        encoding="utf-8"
    )

    for topic in (
        "/zed/zed_node/depth/depth_registered",
        "/zed/zed_node/rgb/color/rect/image",
    ):
        assert topic in script
    assert 'ros2 topic info "$topic_name" --verbose' in script
    assert "Publisher\\ count:\\ [1-9][0-9]*" in script
    assert "Subscription\\ count:\\ [1-9][0-9]*" in script
    assert "Node name: zed_node" in script
    assert "Node name: nvblox_node" in script

    assert "sensor_msgs/msg/CameraInfo --field header.frame_id --once" in script
    assert "zed_left_camera_frame_optical" in script
    assert "geometry_msgs/msg/PoseStamped --field header.frame_id --once" in script
    assert "nav_msgs/msg/Odometry --field child_frame_id --once" in script
    assert "ZED odometry child_frame_id must be zed_camera_link" in script
    assert "/b2/odom" in script
    assert "Node name: odometry_adapter" in script
    assert "B2 odometry child_frame_id must be base_link" in script
    assert "ros2 run tf2_ros tf2_echo map base_link" in script


def test_runtime_check_rejects_default_costmap_fallback() -> None:
    script = (ROOT / "scripts" / "check_isaac_ros_runtime.sh").read_text(
        encoding="utf-8"
    )

    assert 'ros2 param get "$costmap_node" plugins' in script
    assert "for costmap_node in /local_costmap/local_costmap" in script
    assert "nvblox_layer" in script
    assert "inflation_layer" in script
    assert "static_layer" in script
    assert "obstacle_layer" in script
    assert "nvblox_layer.nvblox_map_slice_topic" in script
