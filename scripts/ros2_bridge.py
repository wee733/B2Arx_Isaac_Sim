"""OmniGraph ROS2 接线 (B2ARX D455 → Thor → 回流). 不依赖 Python rclpy。

Isaac Sim 自带 jazzy ROS2 库, C++ plugin 直接用; 在 py3.11 的 isaaclab 环境里
import rclpy 会因 ABI 不匹配崩溃, 所以发布与订阅全部走 OmniGraph 节点。
omni.graph.core 只在 isaacsim 运行时存在, 所以建图函数里才 import, 模块顶层
保持纯逻辑可单测。
"""
from __future__ import annotations

# --- topic / frame 契约 (spec §4 冻结, 两侧硬边界, 改名即断链) ---
COLOR_IMAGE_TOPIC = "/b2arx/d455/color/image_rect"
COLOR_INFO_TOPIC = "/b2arx/d455/color/camera_info"
CLOCK_TOPIC = "/clock"
COLOR_OPTICAL_FRAME = "d455_color_optical_frame"

# --- tag 标识: AprilTag 往 /tf 发 "<family>:<id>" 这个 child frame ---
TAG_FAMILY = "tag36h11"
TAG_ID = 0
TAG_TF_CHILD_FRAME = f"{TAG_FAMILY}:{TAG_ID}"

# 默认 marker prim (num_envs=1; spec §7 R6 多 env 不在 V1 范围)
DEFAULT_TAG_MARKER_PRIM = "/World/envs/env_0/TagMarker"


def build_tag_frame_names_map(marker_prim_path: str) -> list[str]:
    """构造 ROS2SubscribeTransformTree.frameNamesMap。

    顺序是 [prim_path, frame_id, ...] (偶数长度)。本机验证来源:
    docs/ogn/OgnROS2SubscribeTransformTree.rst:41 ("[prim_path_0, frame_name_0, ...]")
    + tests/test_subscribers.py:555 (["/World","world","/World/cube","cube"])。
    顺序写反 (frame_id 在前) 会让 TF 套不到 prim, marker 永不动。
    """
    return [marker_prim_path, TAG_TF_CHILD_FRAME]
