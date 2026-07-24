from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "run_isaac_ros.sh"


def _dry_run(*arguments: str) -> str:
    result = subprocess.run(
        [str(LAUNCHER), *arguments],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def test_sim_launcher_sources_upstream_and_selects_sim_profile() -> None:
    output = _dry_run(
        "sim",
        "--dry-run",
        "sim_address:=127.0.0.1",
        "sim_port:=30000",
        "use_rviz:=false",
    )

    assert "Isaac ROS/ZED workspace" in output
    assert "bringup_sim.launch.py" in output
    assert "domain_id:=23" in output
    assert "sim_port:=30000" in output


def test_real_launcher_selects_hesai_overlay_and_profile() -> None:
    hesai_config = (
        Path.home()
        / "hesai_ws"
        / "src"
        / "HesaiLidar_ROS_2.0"
        / "config"
        / "config.yaml"
    )
    output = _dry_run(
        "real",
        "--dry-run",
        "start_hesai:=true",
        f"hesai_config_file:={hesai_config}",
        "use_rviz:=false",
    )

    assert "Hesai workspace" in output
    assert "bringup_real.launch.py" in output
    assert "start_hesai:=true" in output
    assert f"hesai_config_file:={hesai_config}" in output


def test_launcher_temporarily_disables_nounset_for_generated_setup_files() -> None:
    source = LAUNCHER.read_text(encoding="utf-8")

    assert "set +u" in source
    assert 'source "${setup_file}"' in source
    assert "set -u" in source
    assert "ros2 launch b2arx_nav2_bringup" in source
