from __future__ import annotations

import pytest

from scripts import ros2_bridge


def test_topic_contract_constants_match_spec():
    # spec §4 冻结的契约, 两侧硬边界, 改名即断链
    assert ros2_bridge.COLOR_IMAGE_TOPIC == "/b2arx/d455/color/image_rect"
    assert ros2_bridge.COLOR_INFO_TOPIC == "/b2arx/d455/color/camera_info"
    assert ros2_bridge.CLOCK_TOPIC == "/clock"
    assert ros2_bridge.COLOR_OPTICAL_FRAME == "d455_color_optical_frame"


def test_tag_identity_matches_apriltag_tf_child_frame():
    # AprilTag 往 /tf 发 "<family>:<id>"; family/id 决定 child frame 名
    assert ros2_bridge.TAG_FAMILY == "tag36h11"
    assert ros2_bridge.TAG_ID == 0
    assert ros2_bridge.TAG_TF_CHILD_FRAME == "tag36h11:0"


def test_build_tag_frame_names_map_order_is_prim_then_frame():
    # 顺序必须 [prim_path, frame_id] (本机 rst + test_subscribers.py 验证),
    # 反了 marker 永不动
    result = ros2_bridge.build_tag_frame_names_map(
        marker_prim_path="/World/envs/env_0/TagMarker",
    )
    assert result == ["/World/envs/env_0/TagMarker", "tag36h11:0"]
    assert len(result) % 2 == 0  # frameNamesMap 必须偶数长度


def test_setup_functions_exist_and_are_callable():
    # 函数存在且可取到 (顶层 import 不应触发 omni.graph.core import)
    assert callable(ros2_bridge.setup_d455_ros2_publishers)
    assert callable(ros2_bridge.setup_tag_tf_subscriber)


def test_setup_publishers_raises_importerror_without_omni():
    # 裸 python 没有 omni.graph.core; lazy import 在函数体内,
    # 调用时抛 ImportError, 证明 omni 不是顶层依赖 (否则整个模块 import 就崩, Task 1 测试也跑不了)
    with pytest.raises(ImportError):
        ros2_bridge.setup_d455_ros2_publishers(
            color_camera_prim_path="/World/envs/env_0/Robot/R5a_link6/D455/RSD455/Camera_OmniVision_OV9782_Color",
            domain_id=23, width=640, height=480,
        )


def test_setup_subscriber_raises_importerror_without_omni():
    with pytest.raises(ImportError):
        ros2_bridge.setup_tag_tf_subscriber(domain_id=23)
