from __future__ import annotations

from pathlib import Path
import subprocess

import pytest
import yaml

from scripts.simulation_config import (
    SimulationConfigError,
    config_arguments,
    launch_arguments,
)


ROOT = Path(__file__).resolve().parents[1]
SIM_CONFIG_DIR = ROOT / "config" / "simulation"


def _argument_value(arguments: list[str], option: str) -> str:
    index = len(arguments) - 1 - arguments[::-1].index(option)
    return arguments[index + 1]


def test_warehouse_nav2_profile_maps_to_existing_scene_contract():
    arguments = config_arguments(SIM_CONFIG_DIR / "warehouse_nav2.yaml")

    assert Path(_argument_value(arguments, "--robot_usd")) == (
        ROOT / "assets" / "my_B2Arx" / "my_b2arx" / "my_robot.usd"
    ).resolve()
    assert _argument_value(arguments, "--scene_asset") == "warehouse"
    assert "--no_workspace" in arguments
    assert _argument_value(arguments, "--zed_stream_resolution") == "HD1200"
    assert _argument_value(arguments, "--zed_stream_fps") == "30"
    assert _argument_value(arguments, "--zed_stream_port") == "30000"
    assert _argument_value(arguments, "--zed_stream_transport") == "BOTH"
    assert _argument_value(arguments, "--xt32_pointcloud_topic") == "/lidar_points"
    assert _argument_value(arguments, "--xt32_frame_id") == "hesai_lidar"
    assert "--wrist_camera" in arguments
    assert _argument_value(arguments, "--wrist_camera_prim_path") == "auto"
    assert _argument_value(arguments, "--wrist_camera_width") == "1280"
    assert _argument_value(arguments, "--wrist_camera_height") == "720"
    assert _argument_value(arguments, "--wrist_camera_frame_id") == (
        "wrist_camera_color_optical_frame"
    )
    assert _argument_value(arguments, "--wrist_camera_color_topic") == (
        "/wrist_camera/color/image_raw"
    )
    assert _argument_value(arguments, "--wrist_camera_aligned_depth_topic") == (
        "/wrist_camera/aligned_depth_to_color/image_raw"
    )
    assert _argument_value(arguments, "--ros2_domain_id") == "23"
    assert _argument_value(arguments, "--control_mode") == "policy"
    assert Path(_argument_value(arguments, "--deploy_config")) == (
        ROOT / "config" / "policies" / "basic_locomotion.yaml"
    ).resolve()
    assert _argument_value(arguments, "--viewer_camera") == "scene"
    assert "--ros2" in arguments
    assert "--nav2" in arguments
    assert "--print_policy_debug" in arguments
    assert "--headless" not in arguments


def test_minimal_profile_disables_data_paths_without_changing_core_usd():
    arguments = config_arguments(SIM_CONFIG_DIR / "minimal.yaml")

    assert "--no_zed" in arguments
    assert "--no_zed_sdk_stream" in arguments
    assert "--no_xt32_pointcloud" in arguments
    assert "--wrist_camera" not in arguments
    assert "--no_workspace" in arguments
    assert "--ros2" not in arguments
    assert "--nav2" not in arguments
    assert _argument_value(arguments, "--control_mode") == "hold"
    assert _argument_value(arguments, "--scene_asset") == "minimal"


def test_relative_asset_environment_and_policy_paths_follow_the_yaml(tmp_path: Path):
    config_dir = tmp_path / "profiles"
    config_dir.mkdir()
    config = {
        "schema_version": 1,
        "robot_usd": "../robot.usd",
        "environment": {
            "scene_asset": "default",
            "usd": "../environment.usd",
            "workspace": True,
        },
        "zed": {"enabled": False, "sdk_stream": False},
        "xt32": {"enabled": False},
        "ros": {"enabled": False, "domain_id": 7, "nav2": False},
        "policy": {"control_mode": "hold", "profile": "../policy.yaml"},
        "viewer": {"camera": "scene", "headless": True},
    }
    config_path = config_dir / "portable.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    arguments = config_arguments(config_path)

    assert _argument_value(arguments, "--robot_usd") == str((tmp_path / "robot.usd").resolve())
    assert _argument_value(arguments, "--environment_usd") == str(
        (tmp_path / "environment.usd").resolve()
    )
    assert _argument_value(arguments, "--deploy_config") == str((tmp_path / "policy.yaml").resolve())
    assert _argument_value(arguments, "--ros2_domain_id") == "7"
    assert "--headless" in arguments


def test_extra_scene_arguments_are_last_so_argparse_values_override_yaml():
    arguments = launch_arguments(
        SIM_CONFIG_DIR / "warehouse.yaml",
        ("--zed_stream_port", "31000", "--viewer_camera", "zed_left", "--headless"),
    )

    assert arguments[-5:] == [
        "--zed_stream_port",
        "31000",
        "--viewer_camera",
        "zed_left",
        "--headless",
    ]
    assert arguments.index("30000") < arguments.index("31000")


def test_nav2_profile_rejects_a_disabled_required_sensor(tmp_path: Path):
    profile = yaml.safe_load((SIM_CONFIG_DIR / "warehouse_nav2.yaml").read_text())
    profile["zed"]["sdk_stream"] = False
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text(yaml.safe_dump(profile), encoding="utf-8")

    with pytest.raises(SimulationConfigError, match="zed.sdk_stream"):
        config_arguments(config_path)


def test_shell_launcher_dry_run_uses_profile_and_keeps_cli_override_last():
    result = subprocess.run(
        [
            "bash",
            str(ROOT / "scripts" / "run_isaac_sim.sh"),
            "--config",
            str(SIM_CONFIG_DIR / "warehouse_nav2.yaml"),
            "--dry-run",
            "--zed_stream_port",
            "31000",
            "--headless",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "[INFO] ROS_DOMAIN_ID=23" in result.stdout
    command = next(line for line in result.stdout.splitlines() if line.startswith("[DRY-RUN]"))
    assert "isaac_b2arx_scene.py" in command
    assert command.index("--zed_stream_port 30000") < command.index("--zed_stream_port 31000")
    assert command.endswith("--zed_stream_port 31000 --headless")


def test_launcher_remains_a_thin_wrapper_over_the_existing_scene():
    launcher = (ROOT / "scripts" / "run_isaac_sim.sh").read_text(encoding="utf-8")
    helper = (ROOT / "scripts" / "simulation_config.py").read_text(encoding="utf-8")

    assert "isaac_b2arx_scene.py" in launcher
    assert "isaaclab.sh" in launcher
    assert "exec \"${launch_command[@]}\" \"${scene_arguments[@]}\"" in launcher
    assert "SimulationContext" not in helper
    assert "setup_zed_stream_graph" not in helper
    assert "setup_ros2_xt32_pointcloud" not in helper
