from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from scripts.easy_handeye_to_manipulation import (
    DEFAULT_TRACKING_FRAME_ALIASES,
    convert_easy_handeye,
    load_and_convert,
)


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "config" / "arx_r5a_d543if_eih.calib"
BRINGUP = ROOT / "ros_ws" / "src" / "b2arx_nav2_bringup"
GENERATED = BRINGUP / "config" / "wrist_d435i_eye_in_hand.yaml"
WRAPPER = BRINGUP / "launch" / "manipulation_wrist_d435i.launch.py"


def _source_calibration() -> dict:
    return yaml.safe_load(SOURCE.read_text(encoding="utf-8"))


def test_easy_handeye_result_keeps_effector_to_camera_direction() -> None:
    source = _source_calibration()
    converted = load_and_convert(SOURCE)
    transform = converted["base_to_camera"]

    assert converted["schema_version"] == 1
    assert converted["publish"] is True
    assert transform["parent_frame"] == "link6"
    assert transform["child_frame"] == "wrist_camera_link"

    # easy_handeye2 publishes the saved eye-in-hand value directly with the
    # effector as parent. Equality here catches accidental inversion or a
    # camera-internal optical-frame correction being folded into the mount.
    assert transform["translation"] == [
        source["transform"]["translation"][axis] for axis in "xyz"
    ]
    assert transform["rotation"] == [
        source["transform"]["rotation"][axis] for axis in "xyzw"
    ]
    assert yaml.safe_load(GENERATED.read_text(encoding="utf-8")) == converted


def test_generated_manipulation_calibration_is_reproducible() -> None:
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "easy_handeye_to_manipulation.py"),
            "--check",
        ],
        cwd=ROOT,
        check=True,
    )


def test_converter_rejects_an_unsafe_or_incomplete_frame_chain() -> None:
    source = _source_calibration()

    eye_on_base = deepcopy(source)
    eye_on_base["parameters"]["calibration_type"] = "eye_on_base"
    with pytest.raises(ValueError, match="eye_in_hand"):
        convert_easy_handeye(
            eye_on_base,
            tracking_frame_aliases=DEFAULT_TRACKING_FRAME_ALIASES,
        )

    wrong_effector = deepcopy(source)
    wrong_effector["parameters"]["robot_effector_frame"] = "tool0"
    with pytest.raises(ValueError, match="additional rigid transform"):
        convert_easy_handeye(
            wrong_effector,
            tracking_frame_aliases=DEFAULT_TRACKING_FRAME_ALIASES,
        )

    with pytest.raises(ValueError, match="no explicit frame alias"):
        convert_easy_handeye(source, tracking_frame_aliases={})


def test_wrist_manipulation_launch_is_a_thin_upstream_profile() -> None:
    wrapper = WRAPPER.read_text(encoding="utf-8")
    package_xml = (BRINGUP / "package.xml").read_text(encoding="utf-8")

    upstream_lookup = (
        'get_package_share_directory(\n'
        '        "isaac_ros_manipulation_arx_r5a_bringup"'
    )
    assert upstream_lookup in wrapper
    assert '"arx_r5a_apriltag_pick_and_place.launch.py"' in wrapper
    assert "ComposableNode(" not in wrapper
    assert "Node(" not in wrapper
    assert "RewrittenYaml(" in wrapper
    assert "BEHAVIOR_CAMERA_FRAME_PATH" in wrapper
    assert '"pose_estimation.camera_frame_id"' in wrapper
    assert 'param_rewrites={BEHAVIOR_CAMERA_FRAME_PATH: "base_link"}' in wrapper

    assert '"camera_optical_frame": "wrist_camera_color_optical_frame"' in wrapper
    assert '"output_frame": "base_link"' in wrapper
    assert 'default_value="/wrist_camera/color/image_raw"' in wrapper
    assert (
        'default_value="/wrist_camera/aligned_depth_to_color/image_raw"'
        in wrapper
    )
    assert '"camera_calibration_file": LaunchConfiguration(' in wrapper
    assert "wrist_d435i_eye_in_hand.yaml" in wrapper
    assert "wrist_camera_link -> color optical TF" in GENERATED.read_text(
        encoding="utf-8"
    )
    assert (
        "<exec_depend>isaac_ros_manipulation_arx_r5a_bringup</exec_depend>"
        in package_xml
    )
