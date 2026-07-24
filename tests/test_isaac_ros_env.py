from __future__ import annotations

import os
from pathlib import Path

from scripts.isaac_ros_env import (
    build_ros_bridge_environment,
    ros_bridge_requested,
)


def test_ros_bridge_requested_for_ros_and_nav2_only():
    assert ros_bridge_requested(("--ros2",))
    assert ros_bridge_requested(("--headless", "--nav2"))
    assert not ros_bridge_requested(("--headless", "--no_zed"))


def test_bridge_library_path_is_prepended_once(tmp_path: Path):
    bridge_dir = tmp_path / "bridge" / "jazzy" / "lib"
    other_dir = tmp_path / "other"
    environment = {
        "LD_LIBRARY_PATH": os.pathsep.join((str(other_dir), str(bridge_dir))),
    }

    updated = build_ros_bridge_environment(bridge_dir, environment)

    assert updated["LD_LIBRARY_PATH"].split(os.pathsep) == [
        str(bridge_dir.resolve()),
        str(other_dir),
    ]
    assert updated["ROS_DISTRO"] == "jazzy"


def test_existing_ros_distro_is_preserved(tmp_path: Path):
    bridge_dir = tmp_path / "bridge"
    updated = build_ros_bridge_environment(
        bridge_dir,
        {"LD_LIBRARY_PATH": "", "ROS_DISTRO": "custom"},
    )

    assert updated["ROS_DISTRO"] == "custom"
