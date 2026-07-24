from __future__ import annotations

from pathlib import Path
import sys
import types

import pytest
import yaml

from scripts import ros2_bridge, zed_isaac_sim


def test_ros2_clock_and_command_contract_constants_match_spec():
    assert ros2_bridge.CLOCK_TOPIC == "/clock"
    assert ros2_bridge.CMD_VEL_TOPIC == "/cmd_vel"
    assert ros2_bridge.CMD_VEL_HEARTBEAT_TOPIC == "/cmd_vel_heartbeat"
    assert ros2_bridge.CLOCK_GRAPH_PATH == "/World/B2ArxROS2ClockGraph"
    assert ros2_bridge.CMD_VEL_GRAPH_PATH == "/World/B2ArxROS2CmdVelGraph"


def test_wrist_camera_contract_is_shared_by_sim_and_real():
    assert ros2_bridge.WRIST_CAMERA_GRAPH_PATH == "/World/B2ArxWristCameraGraph"
    assert ros2_bridge.WRIST_COLOR_TOPIC == "/wrist_camera/color/image_raw"
    assert ros2_bridge.WRIST_COLOR_INFO_TOPIC == "/wrist_camera/color/camera_info"
    assert ros2_bridge.WRIST_ALIGNED_DEPTH_TOPIC == (
        "/wrist_camera/aligned_depth_to_color/image_raw"
    )
    assert ros2_bridge.WRIST_ALIGNED_DEPTH_INFO_TOPIC == (
        "/wrist_camera/aligned_depth_to_color/camera_info"
    )
    assert ros2_bridge.WRIST_COLOR_OPTICAL_FRAME == (
        "wrist_camera_color_optical_frame"
    )
    assert ros2_bridge.WRIST_SIM_DEPTH_ENCODING == "32FC1"
    assert ros2_bridge.WRIST_SIM_DEPTH_SCALE == pytest.approx(1.0)


def test_wrist_camera_graph_uses_one_color_render_product_for_rgb_and_depth(
    monkeypatch,
):
    calls = []

    class Keys:
        CREATE_NODES = "create_nodes"
        CONNECT = "connect"
        SET_VALUES = "set_values"

    class Controller:
        @staticmethod
        def edit(graph, specification):
            calls.append((graph, specification))

    Controller.Keys = Keys

    omni_module = types.ModuleType("omni")
    graph_module = types.ModuleType("omni.graph")
    core_module = types.ModuleType("omni.graph.core")
    core_module.Controller = Controller
    graph_module.core = core_module
    omni_module.graph = graph_module
    monkeypatch.setitem(sys.modules, "omni", omni_module)
    monkeypatch.setitem(sys.modules, "omni.graph", graph_module)
    monkeypatch.setitem(sys.modules, "omni.graph.core", core_module)

    result = ros2_bridge.setup_ros2_wrist_camera(
        42,
        "/World/Robot/R5a_link6/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
    )

    assert result == ros2_bridge.WRIST_CAMERA_GRAPH_PATH
    assert len(calls) == 1
    graph, specification = calls[0]
    assert graph == {
        "graph_path": ros2_bridge.WRIST_CAMERA_GRAPH_PATH,
        "evaluator_name": "execution",
    }
    nodes = dict(specification[Keys.CREATE_NODES])
    assert nodes["CreateRenderProduct"] == (
        "isaacsim.core.nodes.IsaacCreateRenderProduct"
    )
    assert nodes["PublishColor"] == "isaacsim.ros2.bridge.ROS2CameraHelper"
    assert nodes["PublishDepth"] == "isaacsim.ros2.bridge.ROS2CameraHelper"
    assert nodes["PublishColorInfo"] == (
        "isaacsim.ros2.bridge.ROS2CameraInfoHelper"
    )
    assert nodes["PublishDepthInfo"] == (
        "isaacsim.ros2.bridge.ROS2CameraInfoHelper"
    )
    values = dict(specification[Keys.SET_VALUES])
    assert values["PublishColor.inputs:type"] == "rgb"
    assert values["PublishDepth.inputs:type"] == "depth"
    assert values["PublishColor.inputs:topicName"] == ros2_bridge.WRIST_COLOR_TOPIC
    assert values["PublishDepth.inputs:topicName"] == (
        ros2_bridge.WRIST_ALIGNED_DEPTH_TOPIC
    )
    assert values["SensorDataQoS.inputs:reliability"] == "bestEffort"
    connections = set(specification[Keys.CONNECT])
    for publisher in (
        "PublishColor",
        "PublishDepth",
        "PublishColorInfo",
        "PublishDepthInfo",
    ):
        assert (
            "CreateRenderProduct.outputs:renderProductPath",
            f"{publisher}.inputs:renderProductPath",
        ) in connections


def test_wrist_camera_graph_rejects_false_interface_contracts_before_kit_import():
    with pytest.raises(ValueError, match="must be absolute"):
        ros2_bridge.setup_ros2_wrist_camera(23, "relative/camera")
    with pytest.raises(ValueError, match="topics must be unique"):
        ros2_bridge.setup_ros2_wrist_camera(
            23,
            "/World/Camera",
            color_topic="/same",
            color_info_topic="/same",
        )


def test_official_zed_release_is_pinned_for_isaac_sim_5_1():
    assert zed_isaac_sim.ZED_ISAAC_SIM_VERSION == "v4.3.0"
    assert zed_isaac_sim.ZED_ISAAC_SIM_RELEASE_URL.startswith(
        "https://github.com/stereolabs/zed-isaac-sim/releases/download/v4.3.0/"
    )
    assert zed_isaac_sim.ZED_ISAAC_SIM_RELEASE_SHA256 == (
        "07e6ef3d6b667152213fc6e3ed324202bb2cbf5bc606d7551c005ebff9271486"
    )
    assert zed_isaac_sim.ZED_EXTENSION_ID == "sl.sensor.camera"
    assert zed_isaac_sim.ZED_EXTENSION_FULL_ID == "sl.sensor.camera-4.3.0"
    assert zed_isaac_sim.ZED_HELPER_NODE_TYPE == "sl.sensor.camera.ZED_Camera"


def test_official_zed_x_geometry_uses_requested_mounting_point():
    zed_isaac_sim.validate_zed_configuration()
    root = (0.280, 0.0, -0.020)
    left = tuple(a + b for a, b in zip(root, zed_isaac_sim.ZED_X_LEFT_CAMERA_OFFSET))
    right = tuple(a + b for a, b in zip(root, zed_isaac_sim.ZED_X_RIGHT_CAMERA_OFFSET))
    assert left == pytest.approx((0.295, 0.06, -0.005))
    assert right == pytest.approx((0.295, -0.06, -0.005))
    assert zed_isaac_sim.ZED_X_BASELINE_M == pytest.approx(0.12)
    assert zed_isaac_sim.ZED_CORE_ROOT_REL_PATH == "b2_description/R5a/ZED_X"
    assert zed_isaac_sim.ZED_X_IMU_PRIM == "base_link/ZED_X/Imu_Sensor"


def test_zed_physics_sensor_is_enabled_in_kit_startup_args_once():
    args = zed_isaac_sim.add_zed_startup_kit_args(
        "--enable isaacsim.sensors.physics --/app/fastShutdown=1"
    )
    assert args.count("isaacsim.sensors.physics") == 1
    assert "isaacsim.robot_setup.assembler" not in args
    assert "--/app/fastShutdown=1" in args


def test_zed_stream_contract_matches_official_nvblox_path():
    assert zed_isaac_sim.stream_dimensions("HD1200") == (1920, 1200)
    assert zed_isaac_sim.stream_intrinsics("HD1200") == pytest.approx((741.6, 741.6, 960.0, 600.0))
    assert zed_isaac_sim.ZED_STREAM_RESOLUTION == "HD1200"
    assert zed_isaac_sim.ZED_STREAM_FPS == 30
    assert zed_isaac_sim.ZED_STREAM_PORT == 30000
    assert zed_isaac_sim.ZED_STREAM_TRANSPORT == "BOTH"
    zed_isaac_sim.validate_zed_stream_settings("SVGA", 120, 30000, "BOTH")
    with pytest.raises(ValueError, match="HD1200@120"):
        zed_isaac_sim.validate_zed_stream_settings("HD1200", 120, 30000, "BOTH")
    assert zed_isaac_sim.ZED_WRAPPER_TOPICS == {
        "depth": "/zed/zed_node/depth/depth_registered",
        "depth_info": "/zed/zed_node/depth/camera_info",
        "color": "/zed/zed_node/rgb/color/rect/image",
        "color_info": "/zed/zed_node/rgb/camera_info",
        "pose": "/zed/zed_node/pose",
    }


def test_zed_wrapper_override_keeps_release_4_5_algorithms():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "ros_ws"
        / "src"
        / "b2arx_nav2_bringup"
        / "config"
        / "zedx_nvblox_release_4_5.yaml"
    )
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
    assert params["general"] == {
        "pub_resolution": "CUSTOM",
        "pub_downscale_factor": 2.0,
        "pub_frame_rate": 30.0,
    }
    assert params["depth"]["depth_mode"] == "ULTRA"
    assert params["depth"]["openni_depth_mode"] is False
    assert params["depth"]["min_depth"] == pytest.approx(0.5)
    assert params["depth"]["max_depth"] == pytest.approx(5.0)
    assert params["pos_tracking"]["pos_tracking_mode"] == "GEN_1"
    assert params["mapping"]["mapping_enabled"] is False
    assert params["debug"]["disable_nitros"] is True


def test_nav2_bringup_keeps_official_nvblox_navigation_algorithms():
    root = Path(__file__).resolve().parents[1]
    config_path = root / "ros_ws" / "src" / "b2arx_nav2_bringup" / "config" / "b2arx_nav2.yaml"
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    controller = params["controller_server"]["ros__parameters"]
    assert controller["odom_topic"] == "/b2/odom"
    rpp = controller["FollowPath"]
    assert rpp["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert rpp["desired_linear_vel"] == pytest.approx(0.25)
    assert rpp["min_approach_linear_velocity"] == pytest.approx(0.25)
    assert rpp["regulated_linear_scaling_min_speed"] == pytest.approx(0.25)
    assert rpp["use_rotate_to_heading"] is False
    assert rpp["allow_reversing"] is False
    assert rpp["use_fixed_curvature_lookahead"] is False
    assert rpp["curvature_lookahead_dist"] == pytest.approx(0.80)
    assert rpp["interpolate_curvature_after_goal"] is False
    assert controller["publish_zero_velocity"] is True
    assert controller["goal_checker_plugins"] == ["position_goal_checker"]
    assert controller["position_goal_checker"] == {
        "plugin": "nav2_controller::PositionGoalChecker",
        "xy_goal_tolerance": pytest.approx(0.30),
        "stateful": True,
    }

    velocity_smoother = params["velocity_smoother"]["ros__parameters"]
    assert velocity_smoother["max_velocity"] == pytest.approx([0.25, 0.0, 0.60])
    assert velocity_smoother["min_velocity"] == pytest.approx([0.0, 0.0, -0.60])
    assert velocity_smoother["max_accel"] == pytest.approx([5.0, 0.0, 6.0])
    assert velocity_smoother["max_decel"] == pytest.approx([-5.0, 0.0, -6.0])

    behavior_server = params["behavior_server"]["ros__parameters"]
    assert behavior_server["max_rotational_vel"] == pytest.approx(0.30)
    assert behavior_server["min_rotational_vel"] == pytest.approx(0.25)
    assert behavior_server["rotational_acc_lim"] == pytest.approx(0.3)
    assert behavior_server["transform_tolerance"] == pytest.approx(0.5)
    assert behavior_server["local_footprint_topic"] == "/behavior_footprint"
    assert behavior_server["global_footprint_topic"] == "/behavior_footprint"

    behavior_footprint = params["behavior_footprint_publisher"]["ros__parameters"]
    assert behavior_footprint == {
        "frame_id": "base_link",
        "points": [0.47, 0.31, 0.47, -0.31, -0.47, -0.31, -0.47, 0.31],
        "publish_rate_hz": 5.0,
    }

    watchdog = params["cmd_vel_watchdog"]["ros__parameters"]
    assert watchdog == {"input_timeout_s": 0.5, "publish_rate_hz": 20.0}

    planner = params["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_smac_planner::SmacPlanner2D"
    assert planner["use_final_approach_orientation"] is True
    assert planner["cost_travel_multiplier"] == pytest.approx(2.0)

    # Both costmaps consume the Nvblox slice in map. Keeping the rolling local
    # costmap in map avoids a non-planar ZED map->odom transform at the plugin
    # boundary without flattening the 3D pose used by Nvblox.
    for costmap_name, frame in (("local_costmap", "map"), ("global_costmap", "map")):
        costmap = params[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["global_frame"] == frame
        assert costmap["robot_base_frame"] == "base_link"
        assert costmap["footprint_padding"] == pytest.approx(0.01)
        assert costmap["plugins"] == ["nvblox_layer", "inflation_layer"]
        assert costmap["nvblox_layer"] == {
            "plugin": "nvblox::nav2::NvbloxCostmapLayer",
            "enabled": True,
            "nav2_costmap_global_frame": frame,
            "nvblox_map_slice_topic": "/nvblox_node/static_map_slice",
            "convert_to_binary_costmap": True,
        }

    launch_dir = (
        root / "ros_ws" / "src" / "b2arx_nav2_bringup" / "launch"
    )
    main_launch = (launch_dir / "isaac_ros_nav2.launch.py").read_text(
        encoding="utf-8"
    )
    zed_launch = (launch_dir / "zed.launch.py").read_text(encoding="utf-8")
    nvblox_launch = (launch_dir / "nvblox.launch.py").read_text(
        encoding="utf-8"
    )
    nav2_launch = (launch_dir / "nav2.launch.py").read_text(encoding="utf-8")
    platform_launch = (launch_dir / "platform_adapters.launch.py").read_text(
        encoding="utf-8"
    )

    # Project launch files must compose the official algorithm launch APIs,
    # not duplicate their ComposableNode/plugin implementations.
    assert '"launch", "zed_camera.launch.py"' in zed_launch
    assert '"ros_params_override_path": zed_params_file' in zed_launch
    assert '"enable_ipc": "false"' in zed_launch
    assert '"debug.disable_nitros:="' in zed_launch
    assert '"simulation.sim_address"' not in zed_launch
    assert "ComposableNode(" not in zed_launch

    assert '"perception",\n                "nvblox.launch.py"' in nvblox_launch
    assert '"run_standalone": IfElseSubstitution(' in nvblox_launch
    assert 'if_value="True"' in nvblox_launch
    assert 'else_value="False"' in nvblox_launch
    assert '"run_standalone": "false"' not in nvblox_launch
    assert "SetParametersFromFile(container_params_file)" in nvblox_launch
    assert "SetParametersFromFile(nvblox_params_file)" in nvblox_launch
    assert '"camera": zed_camera_model' in nvblox_launch
    assert "nvblox::NvbloxNode" not in nvblox_launch

    assert '"launch", "navigation_launch.py"' in nav2_launch
    assert 'official_use_composition = IfElseSubstitution(' in nav2_launch
    assert '"use_composition": official_use_composition' in nav2_launch
    assert '"use_composition",\n                default_value="false"' in nav2_launch
    assert '"use_composition": "true"' in main_launch
    assert "OpaqueFunction(function=_validate_shared_container_contract)" in main_launch
    assert 'executable="cmd_vel_watchdog"' in nav2_launch
    assert 'executable="behavior_footprint_publisher"' in nav2_launch
    assert 'executable="odometry_adapter"' not in nav2_launch
    assert 'executable="odometry_adapter"' in platform_launch
    assert 'LaunchConfiguration("platform_config_file")' in platform_launch
    assert '("footprint_out", "behavior_footprint")' in nav2_launch
    assert '("cmd_vel_in", "cmd_vel_smoothed")' in nav2_launch
    assert '("cmd_vel_out", "cmd_vel")' in nav2_launch
    assert '("heartbeat_out", "cmd_vel_heartbeat")' in nav2_launch
    assert "navigate_w_replanning_time.xml" in nav2_launch

    assert 'package="rclcpp_components"' not in main_launch
    assert 'executable="component_container_mt"' not in main_launch
    assert 'executable="component_container_isolated"' not in main_launch
    assert '"container_params_file": rewritten_nav_params' in main_launch
    assert 'executable="component_container_isolated"' in nvblox_launch
    assert nvblox_launch.count('executable="component_container_isolated"') == 1
    assert '"--use_multi_threaded_executor"' in nvblox_launch
    assert "NVBLOX_CONTAINER_NAME" in main_launch
    assert '"RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")' in main_launch

    package_xml = (
        root
        / "ros_ws"
        / "src"
        / "b2arx_nav2_bringup"
        / "package.xml"
    ).read_text(encoding="utf-8")
    for dependency in (
        "isaac_ros_managed_nitros",
        "isaac_ros_nitros_image_type",
        "nav2_common",
        "nvblox_examples_bringup",
        "nvblox_ros_python_utils",
        "zed_components",
        "zed_msgs",
        "xacro",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_xml

    nvblox_override = yaml.safe_load(
        (
            root
            / "ros_ws"
            / "src"
            / "b2arx_nav2_bringup"
            / "config"
            / "nvblox_b2arx.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert nvblox_override["global_frame"] == "map"
    assert nvblox_override["num_cameras"] == 1
    assert nvblox_override["use_depth"] is True
    assert nvblox_override["use_lidar"] is False
    assert 'LaunchConfiguration("use_lidar_in_nvblox")' in nvblox_launch
    assert "OpaqueFunction(function=_validate_sensor_contract)" in nvblox_launch
    assert "pointcloud" not in nvblox_launch
    assert not (
        root
        / "ros_ws"
        / "src"
        / "b2arx_nav2_bringup"
        / "config"
        / "nvblox_base_release_4_5.yaml"
    ).exists()


def test_isaac_ros_preflight_rejects_private_overlay_and_requires_nitros():
    root = Path(__file__).resolve().parents[1]
    script = (root / "scripts" / "check_isaac_ros_4_5.sh").read_text(encoding="utf-8")
    assert "isaac-ros activate" in script
    assert "VIRTUAL_ENV" in script
    assert "apt_root" in script
    assert "/.local/share/b2arx_isaac_ros/zed_sdk_" in script
    assert "rmw_cyclonedds_cpp" in script
    assert 'ctypes.CDLL("librmw_cyclonedds_cpp.so")' in script
    assert "isaac_ros_launch_utils" in script
    assert "import isaac_ros_launch_utils" in script
    versioned_45_packages = script.split("for package_name in \\\n", 1)[1].split("; do", 1)[0]
    assert "isaac_ros_launch_utils" in versioned_45_packages
    assert 'require_version_prefix "$package_name" "4.5"' in script
    assert "libisaac_ros_nitros.so" in script
    assert "libisaac_ros_nitros_image_type.so" in script
    assert "require_exact_linkage" in script
    assert 'Shared library: [$dependency_name]' in script
    assert "readelf -d" in script
    assert "not found" in script
    assert 'ZED_SDK_ROOT="/usr/local/zed"' in script
    assert "zed-config-version.cmake" in script
    assert '"libsl_zed.so" "$zed_sdk_library"' in script
    assert "|| true" not in script
    assert 'require_version_prefix zed_components "5.4"' in script
    assert 'require_version_prefix zed_msgs "5.3"' in script
    assert "nvblox_examples_bringup" in script


def test_isaac_ros_runtime_check_covers_zed_transport_nvblox_and_nav2():
    root = Path(__file__).resolve().parents[1]
    script = (root / "scripts" / "check_isaac_ros_runtime.sh").read_text(
        encoding="utf-8"
    )

    # ZED keeps the lifecycle owned by its official launch. Nvblox and Nav2
    # share the project-created algorithm container.
    assert "ros2 component list /zed/zed_container" in script
    assert "ros2 component list /nvblox_container" in script
    assert "'/zed/zed_node'" in script
    assert "'/nvblox_node'" in script

    # The stable default uses normal ROS images, while retaining a checked
    # opt-in NITROS A/B path for a future SDK/driver update.
    assert "ros2 param get /zed/zed_node debug.disable_nitros" in script
    assert '[[ "$nitros_parameter" == *True* ]]' in script
    assert 'zed_nitros_enabled=false' in script
    assert '[[ "$nitros_parameter" == *False* ]]' in script
    assert 'zed_nitros_enabled=true' in script
    assert 'if [[ "$zed_nitros_enabled" == true ]]' in script
    for topic_name in (
        "/zed/zed_node/depth/depth_registered/nitros",
        "/zed/zed_node/rgb/color/rect/image/nitros",
    ):
        assert topic_name in script

    # A published map slice proves that Nvblox is receiving and integrating data.
    assert "/nvblox_node/static_map_slice" in script
    assert "/lidar_points" in script
    assert "hesai_lidar" in script
    assert "sensor_msgs/msg/PointCloud2" in script
    assert "header.frame_id" in script
    assert "fields" in script
    assert "base_link hesai_lidar" in script
    assert "ros2 topic echo" in script
    assert "--once" in script
    assert "/b2/odom" in script
    assert "Node name: odometry_adapter" in script
    assert "B2 odometry child_frame_id must be base_link" in script

    # The runtime check must verify the core composed Nav2 lifecycle nodes are active.
    assert "ros2 lifecycle get" in script
    assert "active [3]" in script
    for lifecycle_node in (
        "controller_server",
        "planner_server",
        "bt_navigator",
        "velocity_smoother",
    ):
        assert lifecycle_node in script

    # Active lifecycle nodes are insufficient if dynamically-created costmap
    # nodes silently inherited Nav2 defaults instead of the Nvblox layers.
    assert 'ros2 param get "$costmap_node" plugins' in script
    assert "nvblox_layer.nvblox_map_slice_topic" in script
    assert "fell back to Nav2 default layers" in script

    # No Nav2 auxiliary server may bypass the policy-aware watchdog.
    assert "ros2 topic info /cmd_vel --verbose" in script
    assert "Publisher count: 1" in script
    assert "Node name: cmd_vel_watchdog" in script


def test_readme_uses_thin_official_workspace_launchers():
    root = Path(__file__).resolve().parents[1]
    readme = (root / "README.md").read_text(encoding="utf-8")
    assert "./scripts/run_isaac_sim.sh" in readme
    assert "./scripts/run_isaac_ros.sh sim" in readme
    assert "./scripts/run_isaac_ros.sh real" in readme
    assert "scripts/check_isaac_ros_4_5.sh" in readme
    assert "zed_wrapper/launch/zed_camera.launch.py" in readme
    assert "nvblox_examples_bringup/launch/perception/nvblox.launch.py" in readme
    assert "nav2_bringup/launch/navigation_launch.py" in readme
    assert "sim_address" in readme
    assert "Isaac Sim 主机可达地址" in readme
    assert "APT_ROOT" not in readme
    assert "apt_root" not in readme


def test_zed_sim_address_targets_streamer_host_and_supports_env_or_cli_override():
    root = Path(__file__).resolve().parents[1]
    launch_dir = (
        root
        / "ros_ws"
        / "src"
        / "b2arx_nav2_bringup"
        / "launch"
    )
    main_launch = (launch_dir / "isaac_ros_nav2.launch.py").read_text(
        encoding="utf-8"
    )
    zed_launch = (launch_dir / "zed.launch.py").read_text(encoding="utf-8")
    scene_text = (root / "scripts" / "isaac_b2arx_scene.py").read_text(
        encoding="utf-8"
    )

    assert 'LaunchConfiguration("sim_address")' in main_launch
    assert '"sim_address": sim_address' in zed_launch
    assert '"SIM_ADDRESS", default_value="127.0.0.1"' in main_launch
    assert "Isaac Sim host's reachable Ethernet address" in main_launch
    assert 'EnvironmentVariable("ZED_PORT", default_value="30000")' in main_launch
    assert '"sim_mode": PythonExpression(' in zed_launch

    # sim_address is a zed_wrapper receiver parameter. The Isaac Sim streamer
    # itself only selects its transport and port, so do not add a misleading
    # receiver/Thor address option to the simulator CLI.
    assert '"--zed_stream_port"' in scene_text
    assert '"--zed_stream_transport"' in scene_text
    assert "--zed_stream_address" not in scene_text


def test_ros_bringup_zed_override_is_the_single_runtime_contract():
    root = Path(__file__).resolve().parents[1]
    duplicate = root / "config" / "zed" / "zedx_nvblox_release_4_5.yaml"
    runtime = yaml.safe_load(
        (
            root
            / "ros_ws"
            / "src"
            / "b2arx_nav2_bringup"
            / "config"
            / "zedx_nvblox_release_4_5.yaml"
        ).read_text()
    )
    assert not duplicate.exists()
    assert runtime["/**"]["ros__parameters"]["debug"]["disable_nitros"] is True


def test_nav2_tf_seam_uses_inverse_official_mount_without_second_camera_parent():
    root = Path(__file__).resolve().parents[1]
    launch_text = (
        root
        / "ros_ws"
        / "src"
        / "b2arx_nav2_bringup"
        / "launch"
        / "platform_adapters.launch.py"
    ).read_text(encoding="utf-8")
    platform_config = yaml.safe_load(
        (
            root
            / "ros_ws"
            / "src"
            / "b2arx_nav2_bringup"
            / "config"
            / "platform_adapters.yaml"
        ).read_text(encoding="utf-8")
    )
    zed_mount = platform_config["mounts"]["zed_camera_to_base"]
    assert zed_mount == {
        "parent_frame": "zed_camera_link",
        "child_frame": "base_link",
        "translation": pytest.approx([-0.525, 0.0, -0.079]),
        "quaternion_xyzw": pytest.approx([0.0, 0.0, 0.0, 1.0]),
    }
    assert "USD root -> left camera" in launch_text
    assert "zed_wrapper camera_link -> left camera" in launch_text
    assert '"--frame-id"' in launch_text
    assert '"--child-frame-id"' in launch_text
    assert 'executable="odometry_adapter"' in launch_text
    assert '"camera_to_base_translation": list(zed_mount["translation"])' in launch_text
    assert '"camera_to_base_quaternion": list(zed_mount["quaternion"])' in launch_text


def test_zed_installation_validator_uses_override_root(monkeypatch, tmp_path):
    monkeypatch.setenv("ZED_ISAAC_SIM_ROOT", str(tmp_path))
    extension = tmp_path / zed_isaac_sim.ZED_ISAAC_SIM_BUILD_DIRNAME / "exts" / "sl.sensor.camera"
    relative_paths = (
        "config/extension.toml",
        "bin/libsl.sensor.camera.plugin.so",
        "data/usd/ZED_X.usdc",
    )
    test_digest = "9f86d081884c7d659a2feaa0c55ad015a3bf4f1b2b0b822cd15d6c15b0f00a08"
    monkeypatch.setattr(
        zed_isaac_sim,
        "ZED_EXTENSION_FILE_SHA256",
        {relative: test_digest for relative in relative_paths},
    )
    for relative in relative_paths:
        path = extension / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"test")
    zed_isaac_sim.validate_zed_installation()


def test_zed_stream_graph_rejects_invalid_settings_before_kit_import():
    with pytest.raises(ValueError, match="positive and even"):
        zed_isaac_sim.setup_zed_stream_graph("/World/ZED_X", port=30001)


def test_setup_functions_exist_and_are_callable():
    # 函数存在且可取到 (顶层 import 不应触发 omni.graph.core import)
    assert callable(ros2_bridge.setup_ros2_clock)
    assert callable(ros2_bridge.setup_ros2_cmd_vel_subscriber)


def test_setup_graph_functions_raise_importerror_without_omni():
    # 裸 python 没有 omni.graph.core; lazy import 在函数体内,
    # 调用时抛 ImportError, 证明 omni 不是顶层依赖 (否则整个模块 import 就崩, Task 1 测试也跑不了)
    with pytest.raises(ImportError):
        ros2_bridge.setup_ros2_clock(domain_id=23)
    with pytest.raises(ImportError):
        ros2_bridge.setup_ros2_cmd_vel_subscriber(domain_id=23)


def test_cmd_vel_graph_rejects_relative_topic_before_omni_import():
    with pytest.raises(ValueError, match="absolute"):
        ros2_bridge.setup_ros2_cmd_vel_subscriber(domain_id=23, topic_name="cmd_vel")
    with pytest.raises(ValueError, match="heartbeat topic must be absolute"):
        ros2_bridge.setup_ros2_cmd_vel_subscriber(
            domain_id=23,
            heartbeat_topic_name="cmd_vel_heartbeat",
        )


def test_xt32_pointcloud_contract_constants_and_validation():
    assert ros2_bridge.XT32_POINTCLOUD_GRAPH_PATH == "/World/B2ArxXT32PointCloudGraph"
    assert ros2_bridge.XT32_POINTCLOUD_TOPIC == "/lidar_points"
    assert ros2_bridge.XT32_FRAME_ID == "hesai_lidar"
    with pytest.raises(ValueError, match="absolute"):
        ros2_bridge.setup_ros2_xt32_pointcloud(
            domain_id=23,
            lidar_prim_path="World/XT32",
        )
    with pytest.raises(ValueError, match="point cloud topic must be absolute"):
        ros2_bridge.setup_ros2_xt32_pointcloud(
            domain_id=23,
            lidar_prim_path="/World/XT32",
            topic_name="lidar_points",
        )
    with pytest.raises(ValueError, match="frame id"):
        ros2_bridge.setup_ros2_xt32_pointcloud(
            domain_id=23,
            lidar_prim_path="/World/XT32",
            frame_id="/hesai_lidar",
        )


def test_xt32_pointcloud_graph_is_lazy_and_uses_full_scan_xyz_writer():
    with pytest.raises(ImportError):
        ros2_bridge.setup_ros2_xt32_pointcloud(
            domain_id=23,
            lidar_prim_path="/World/envs/env_0/Robot/b2_description/XT_32/PandarXT_32_10hz",
        )

    source = Path(ros2_bridge.__file__).read_text(encoding="utf-8")
    assert "OgnIsaacRunOneSimulationFrame" in source
    assert '"PublishPointCloud", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"' in source
    assert '("PublishPointCloud.inputs:type", "point_cloud")' in source
    assert '("PublishPointCloud.inputs:fullScan", bool(full_scan))' in source
    assert '("PublishPointCloud.inputs:useSystemTime", False)' in source
    assert '("PublishPointCloud.inputs:resetSimulationTimeOnStop", False)' in source
