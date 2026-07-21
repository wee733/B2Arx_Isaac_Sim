from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from scripts import ros2_bridge, zed_isaac_sim


def test_topic_contract_constants_match_spec():
    # spec §4 冻结的契约, 两侧硬边界, 改名即断链
    assert ros2_bridge.COLOR_IMAGE_TOPIC == "/b2arx/d455/color/image_rect"
    assert ros2_bridge.COLOR_INFO_TOPIC == "/b2arx/d455/color/camera_info"
    assert ros2_bridge.CLOCK_TOPIC == "/clock"
    assert ros2_bridge.COLOR_OPTICAL_FRAME == "d455_color_optical_frame"
    assert ros2_bridge.CMD_VEL_TOPIC == "/cmd_vel"
    assert ros2_bridge.CMD_VEL_HEARTBEAT_TOPIC == "/cmd_vel_heartbeat"
    assert ros2_bridge.CMD_VEL_GRAPH_PATH == "/World/B2ArxROS2CmdVelGraph"


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
    expected = zed_isaac_sim.expected_camera_positions_in_r5a()
    assert expected["left"] == pytest.approx((0.295, 0.06, -0.015))
    assert expected["right"] == pytest.approx((0.295, -0.06, -0.015))
    assert zed_isaac_sim.ZED_X_BASELINE_M == pytest.approx(0.12)
    assert zed_isaac_sim.ZED_X_ASSEMBLER_MOUNT_PRIM == "base_link"
    assert zed_isaac_sim.ZED_X_IMU_PRIM == "base_link/ZED_X/Imu_Sensor"


def test_zed_physics_sensor_is_enabled_in_kit_startup_args_once():
    args = zed_isaac_sim.add_zed_startup_kit_args(
        "--enable isaacsim.sensors.physics --/app/fastShutdown=1"
    )
    assert args.count("isaacsim.sensors.physics") == 1
    assert "--enable isaacsim.robot_setup.assembler" in args
    assert "--/app/fastShutdown=1" in args


def test_zed_stream_contract_matches_official_nvblox_path():
    assert zed_isaac_sim.stream_dimensions("HD1200") == (1920, 1200)
    assert zed_isaac_sim.stream_intrinsics("HD1200") == pytest.approx((741.6, 741.6, 960.0, 600.0))
    assert zed_isaac_sim.ZED_STREAM_RESOLUTION == "HD1200"
    assert zed_isaac_sim.ZED_STREAM_FPS == 30
    assert zed_isaac_sim.ZED_STREAM_PORT == 30000
    zed_isaac_sim.validate_zed_stream_settings("SVGA", 120, 30000, "BOTH")
    with pytest.raises(ValueError, match="HD1200@120"):
        zed_isaac_sim.validate_zed_stream_settings("HD1200", 120, 30000, "BOTH")
    assert zed_isaac_sim.ZED_WRAPPER_TOPICS == {
        "depth": "/zed/zed_node/depth/depth_registered",
        "depth_info": "/zed/zed_node/depth/camera_info",
        "color": "/zed/zed_node/rgb/color/rect/image",
        "color_info": "/zed/zed_node/rgb/color/rect/camera_info",
        "pose": "/zed/zed_node/pose",
    }


def test_zed_wrapper_override_keeps_release_4_5_algorithms():
    config_path = Path(__file__).resolve().parents[1] / "config" / "zed" / "zedx_nvblox_release_4_5.yaml"
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


def test_nav2_bringup_keeps_official_nvblox_navigation_algorithms():
    root = Path(__file__).resolve().parents[1]
    config_path = root / "ros_ws" / "src" / "b2arx_nav2_bringup" / "config" / "b2arx_nav2.yaml"
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    controller = params["controller_server"]["ros__parameters"]
    assert controller["odom_topic"] == "/zed/zed_node/odom"
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

    launch_text = (
        root
        / "ros_ws"
        / "src"
        / "b2arx_nav2_bringup"
        / "launch"
        / "b2arx_zed_nvblox_nav2.launch.py"
    ).read_text(encoding="utf-8")
    assert 'package="nav2_collision_monitor"' not in launch_text
    assert '("cmd_vel_smoothed", "cmd_vel_smoothed")' in launch_text
    assert 'executable="cmd_vel_watchdog"' in launch_text
    assert 'executable="behavior_footprint_publisher"' in launch_text
    assert '("footprint_out", "behavior_footprint")' in launch_text
    assert '("cmd_vel_in", "cmd_vel_smoothed")' in launch_text
    assert '("cmd_vel_out", "cmd_vel")' in launch_text
    assert '("heartbeat_out", "cmd_vel_heartbeat")' in launch_text
    assert '"global_frame": "map"' in launch_text
    assert "TimerAction(" in launch_text
    assert "period=5.0" in launch_text
    assert 'get_package_share_directory("nav2_bt_navigator")' in launch_text
    assert '"default_nav_to_pose_bt_xml": nav_to_pose_bt_xml' in launch_text
    assert "behavior_trees/navigate_w_replanning_time.xml" in launch_text


def test_ros_bringup_zed_override_is_identical_to_root_contract():
    root = Path(__file__).resolve().parents[1]
    source = yaml.safe_load((root / "config" / "zed" / "zedx_nvblox_release_4_5.yaml").read_text())
    installed = yaml.safe_load(
        (
            root
            / "ros_ws"
            / "src"
            / "b2arx_nav2_bringup"
            / "config"
            / "zedx_nvblox_release_4_5.yaml"
        ).read_text()
    )
    assert installed == source


def test_nav2_tf_seam_uses_inverse_official_mount_without_second_camera_parent():
    root = Path(__file__).resolve().parents[1]
    launch_text = (
        root
        / "ros_ws"
        / "src"
        / "b2arx_nav2_bringup"
        / "launch"
        / "b2arx_zed_nvblox_nav2.launch.py"
    ).read_text(encoding="utf-8")
    assert "ZED_CAMERA_LINK_TO_B2_BASE = (-0.500, 0.0, -0.070)" in launch_text
    assert '"--frame-id", "zed_camera_link"' in launch_text
    assert '"--child-frame-id", "base_link"' in launch_text


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


def test_tag_identity_matches_apriltag_tf_child_frame():
    # AprilTag 往 /tf 发 "<family>:<id>"; family/id 决定 child frame 名
    assert ros2_bridge.TAG_FAMILY == "tag36h11"
    assert ros2_bridge.TAG_ID == 0
    assert ros2_bridge.TAG_TF_CHILD_FRAME == "tag36h11:0"


def test_build_tag_frame_names_map_maps_parent_and_child_prim_then_frame():
    # frameNamesMap 每对是 [prim_path, frame_id] (本机 rst + test_subscribers.py 验证),
    # 且 parent(ROS 光学系子 prim) 和 child(tag) 都要在 map 里, 否则节点找不到 parent 参考系 marker 不动。
    optical = "/World/envs/env_0/Robot/R5a_link6/D455/RSD455/Camera_OmniVision_OV9782_Color/ros_optical"
    result = ros2_bridge.build_tag_frame_names_map(
        marker_prim_path="/World/envs/env_0/TagMarker",
        color_optical_prim_path=optical,
    )
    assert result == [
        optical, "d455_color_optical_frame",          # parent: prim 在前, frame 在后
        "/World/envs/env_0/TagMarker", "tag36h11:0",  # child: 同序
    ]
    assert len(result) % 2 == 0  # frameNamesMap 必须偶数长度


def test_setup_functions_exist_and_are_callable():
    # 函数存在且可取到 (顶层 import 不应触发 omni.graph.core import)
    assert callable(ros2_bridge.setup_d455_ros2_publishers)
    assert callable(ros2_bridge.setup_ros2_clock)
    assert callable(ros2_bridge.setup_ros2_cmd_vel_subscriber)
    assert callable(ros2_bridge.setup_tag_tf_subscriber)


def test_setup_publishers_raises_importerror_without_omni():
    # 裸 python 没有 omni.graph.core; lazy import 在函数体内,
    # 调用时抛 ImportError, 证明 omni 不是顶层依赖 (否则整个模块 import 就崩, Task 1 测试也跑不了)
    with pytest.raises(ImportError):
        ros2_bridge.setup_d455_ros2_publishers(
            color_camera_prim_path="/World/envs/env_0/Robot/R5a_link6/D455/RSD455/Camera_OmniVision_OV9782_Color",
            domain_id=23, width=640, height=480,
        )
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


def test_setup_subscriber_raises_importerror_without_omni():
    with pytest.raises(ImportError):
        ros2_bridge.setup_tag_tf_subscriber(domain_id=23)
