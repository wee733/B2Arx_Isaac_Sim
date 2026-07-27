from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / "ros_ws" / "src" / "b2arx_nav2_bringup"


def _launch_text(name: str) -> str:
    return (BRINGUP / "launch" / name).read_text(encoding="utf-8")


def test_xt32_profile_keeps_non_costmap_navigation_policy_in_sync():
    depth = yaml.safe_load(
        (BRINGUP / "config" / "b2arx_nav2.yaml").read_text(encoding="utf-8")
    )
    lidar = yaml.safe_load(
        (BRINGUP / "config" / "b2arx_nav2_xt32.yaml").read_text(
            encoding="utf-8"
        )
    )

    for costmap_name in ("local_costmap", "global_costmap"):
        depth.pop(costmap_name)
        lidar.pop(costmap_name)
    assert lidar == depth


def test_xt32_profile_uses_official_obstacle_layer_for_both_costmaps():
    params = yaml.safe_load(
        (BRINGUP / "config" / "b2arx_nav2_xt32.yaml").read_text(
            encoding="utf-8"
        )
    )

    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = params[costmap_name][costmap_name]["ros__parameters"]
        assert costmap["plugins"] == ["obstacle_layer", "inflation_layer"]
        assert "nvblox_layer" not in costmap["plugins"]
        obstacle = costmap["obstacle_layer"]
        assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
        assert obstacle["observation_sources"] == "xt32"
        assert obstacle["xt32"] == {
            "topic": "/lidar_points",
            "sensor_frame": "hesai_lidar",
            "data_type": "PointCloud2",
            "marking": True,
            "clearing": True,
            "min_obstacle_height": -0.50,
            "max_obstacle_height": 1.50,
            "obstacle_min_range": 0.30,
            "obstacle_max_range": 20.0,
            "raytrace_min_range": 0.20,
            "raytrace_max_range": 25.0,
            "observation_persistence": 0.0,
            "expected_update_rate": 0.0,
        }


def test_xt32_launch_does_not_start_nvblox_and_keeps_external_odom_escape_hatch():
    launch = _launch_text("b2arx_xt32_nav2.launch.py")

    assert '"nvblox.launch.py"' not in launch
    assert '"nav2.launch.py"' in launch
    assert '"hesai_xt32.launch.py"' in launch
    assert '"start_zed_wrapper",\n                default_value="true"' in launch
    assert '"start_odometry_adapter", default_value="true"' in launch
    assert '"use_composition": "false"' in launch


def test_sim_and_real_profiles_select_xt32_mode_without_changing_depth_default():
    sim = _launch_text("bringup_sim.launch.py")
    real = _launch_text("bringup_real.launch.py")

    for profile in (sim, real):
        assert '"navigation_mode"' in profile
        assert 'choices=["depth", "lidar"]' in profile
        assert '"b2arx_xt32_nav2.launch.py"' in profile
        assert '"isaac_ros_nav2.launch.py"' in profile
        assert 'EqualsSubstitution(navigation_mode, "depth")' in profile
        assert 'EqualsSubstitution(navigation_mode, "lidar")' in profile

    assert 'default_value="depth"' in sim
    assert 'default_value="depth"' in real
    for profile in (sim, real):
        assert '"start_zed_wrapper": LaunchConfiguration(' in profile
        assert '"start_odometry_adapter": LaunchConfiguration(' in profile


def test_shell_launcher_passes_the_lidar_mode_to_the_sim_profile():
    result = subprocess.run(
        [
            str(ROOT / "scripts" / "run_isaac_ros.sh"),
            "sim",
            "--dry-run",
            "navigation_mode:=lidar",
            "use_rviz:=false",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "bringup_sim.launch.py" in result.stdout
    assert "navigation_mode:=lidar" in result.stdout


def test_runtime_checker_has_separate_depth_and_lidar_contracts():
    checker = (ROOT / "scripts" / "check_isaac_ros_runtime.sh").read_text(
        encoding="utf-8"
    )

    assert "--navigation-mode" in checker
    assert "navigation_mode=depth" in checker
    assert 'if [[ "$navigation_mode" == depth ]]; then' in checker
    assert 'if [[ "$navigation_mode" == lidar ]]; then' in checker
    assert "obstacle_layer.xt32.topic" in checker
    assert "Nvblox map slice: receiving data" in checker
    assert "Nav2 ObstacleLayer" in checker
