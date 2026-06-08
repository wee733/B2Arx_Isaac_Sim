from __future__ import annotations

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
