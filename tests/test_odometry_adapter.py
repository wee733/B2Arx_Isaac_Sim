from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from ros_ws.src.b2arx_nav2_bringup.b2arx_nav2_bringup.odometry_adapter_core import (
    DEFAULT_CAMERA_TO_BASE_TRANSLATION,
    OdometryState,
    transform_camera_odometry_to_base,
)


ROOT = Path(__file__).resolve().parents[1]
NAV2_PARAMS = (
    ROOT / "ros_ws" / "src" / "b2arx_nav2_bringup" / "config" / "b2arx_nav2.yaml"
)
PLATFORM_PARAMS = (
    ROOT
    / "ros_ws"
    / "src"
    / "b2arx_nav2_bringup"
    / "config"
    / "platform_adapters.yaml"
)


def assert_vector_close(actual, expected) -> None:
    assert actual == pytest.approx(expected, abs=1.0e-12)


def test_default_mount_moves_camera_pose_to_base_origin() -> None:
    result = transform_camera_odometry_to_base(
        OdometryState(
            position=(1.0, 2.0, 3.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        )
    )

    assert_vector_close(result.position, (0.475, 2.0, 2.921))
    assert_vector_close(result.orientation_xyzw, (0.0, 0.0, 0.0, 1.0))


def test_pose_composes_odom_camera_and_camera_base_transforms() -> None:
    half_sqrt = math.sqrt(0.5)
    result = transform_camera_odometry_to_base(
        OdometryState(
            position=(1.0, 2.0, 3.0),
            orientation_xyzw=(0.0, 0.0, half_sqrt, half_sqrt),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0),
        ),
        camera_to_base_translation=(1.0, 0.0, 0.0),
        camera_to_base_quaternion=(half_sqrt, 0.0, 0.0, half_sqrt),
    )

    assert_vector_close(result.position, (1.0, 3.0, 3.0))
    assert_vector_close(result.orientation_xyzw, (0.5, 0.5, 0.5, 0.5))


def test_twist_applies_default_mount_lever_arm() -> None:
    result = transform_camera_odometry_to_base(
        OdometryState(
            position=(0.0, 0.0, 0.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_velocity=(1.0, 2.0, 3.0),
            angular_velocity=(0.0, 0.0, 2.0),
        )
    )

    # v_base_at_camera = v_camera + omega x p_camera_base
    assert DEFAULT_CAMERA_TO_BASE_TRANSLATION == (-0.525, 0.0, -0.079)
    assert_vector_close(result.linear_velocity, (1.0, 0.95, 3.0))
    assert_vector_close(result.angular_velocity, (0.0, 0.0, 2.0))


def test_twist_is_reexpressed_in_base_coordinates() -> None:
    half_sqrt = math.sqrt(0.5)
    result = transform_camera_odometry_to_base(
        OdometryState(
            position=(0.0, 0.0, 0.0),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 1.0),
        ),
        camera_to_base_translation=(1.0, 0.0, 0.0),
        camera_to_base_quaternion=(0.0, 0.0, half_sqrt, half_sqrt),
    )

    assert_vector_close(result.linear_velocity, (1.0, 0.0, 0.0))
    assert_vector_close(result.angular_velocity, (0.0, 0.0, 1.0))


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("orientation_xyzw", (0.0, 0.0, 0.0, 0.0), "zero quaternion"),
        ("linear_velocity", (0.0, float("nan"), 0.0), "finite"),
        ("position", (0.0, 0.0), "3 values"),
    ],
)
def test_invalid_odometry_is_rejected(field, value, message) -> None:
    values = {
        "position": (0.0, 0.0, 0.0),
        "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        "linear_velocity": (0.0, 0.0, 0.0),
        "angular_velocity": (0.0, 0.0, 0.0),
    }
    values[field] = value

    with pytest.raises(ValueError, match=message):
        transform_camera_odometry_to_base(OdometryState(**values))


def test_nav2_consumes_only_base_origin_odometry() -> None:
    params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))

    assert params["bt_navigator"]["ros__parameters"]["odom_topic"] == "/b2/odom"
    assert params["controller_server"]["ros__parameters"]["odom_topic"] == "/b2/odom"
    assert params["velocity_smoother"]["ros__parameters"]["odom_topic"] == "/b2/odom"


def test_platform_profile_is_the_single_mount_source_for_tf_and_odometry() -> None:
    params = yaml.safe_load(PLATFORM_PARAMS.read_text(encoding="utf-8"))
    zed_mount = params["mounts"]["zed_camera_to_base"]
    hesai_mount = params["mounts"]["base_to_hesai_lidar"]
    odometry = params["odometry_adapter"]["ros__parameters"]
    launch_text = (
        PLATFORM_PARAMS.parent.parent / "launch" / "platform_adapters.launch.py"
    ).read_text(encoding="utf-8")

    assert zed_mount["parent_frame"] == "zed_camera_link"
    assert zed_mount["child_frame"] == "base_link"
    assert_vector_close(zed_mount["translation"], DEFAULT_CAMERA_TO_BASE_TRANSLATION)
    assert_vector_close(zed_mount["quaternion_xyzw"], (0.0, 0.0, 0.0, 1.0))
    assert odometry == {
        "input_topic": "/zed/zed_node/odom",
        "output_topic": "/b2/odom",
        "publish_tf": False,
    }
    assert hesai_mount["child_frame"] == "hesai_lidar"
    assert '"camera_to_base_translation": list(zed_mount["translation"])' in launch_text
    assert '"camera_to_base_quaternion": list(zed_mount["quaternion"])' in launch_text
