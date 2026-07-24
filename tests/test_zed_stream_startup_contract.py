from pathlib import Path

from scripts import zed_isaac_sim


ROOT = Path(__file__).resolve().parents[1]


def test_scene_does_not_report_the_streamer_ready_when_only_the_graph_exists():
    scene_source = (ROOT / "scripts" / "isaac_b2arx_scene.py").read_text(
        encoding="utf-8"
    )

    assert "stream graph active" not in scene_source
    assert "stream graph configured " in scene_source
    assert "(streamer is not ready yet): " in scene_source
    assert "ZED Streamer initialized successfully with ID" in scene_source


def test_readme_requires_sender_readiness_before_starting_zed_wrapper():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    assert "stream graph configured" in readme
    assert "ZED Streamer initialized successfully" in readme
    assert "不能作为 ROS receiver 的启动信号" in readme
    assert "两端端口必须一致" in readme


def test_scene_binds_streaming_to_the_embedded_core_usd_sensors():
    scene_source = (ROOT / "scripts" / "isaac_b2arx_scene.py").read_text(
        encoding="utf-8"
    )

    assert zed_isaac_sim.ZED_CORE_ROOT_REL_PATH == "b2_description/R5a/ZED_X"
    assert 'ZED_ASSET_PRIM_PATH = f"{ROBOT_PRIM_PATH}/{ZED_CORE_ROOT_REL_PATH}"' in scene_source
    assert "camera_prim_path=ZED_ASSET_PRIM_PATH" in scene_source
    assert 'XT32_LIDAR_REL_PATH = "b2_description/XT_32/PandarXT_32_10hz"' in scene_source
    assert "setup_ros2_xt32_pointcloud(" in scene_source

    # The core USD owns both sensors. The scene must not recreate the old
    # sibling ZED or assemble another rigid body at runtime.
    assert "zed_asset =" not in scene_source
    assert "attach_zed_x_to_robot" not in scene_source
    assert "RobotAssembler" not in scene_source
