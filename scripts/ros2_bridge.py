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


PUB_GRAPH_PATH = "/World/B2ArxROS2PubGraph"
SUB_GRAPH_PATH = "/World/B2ArxROS2SubGraph"


def setup_d455_ros2_publishers(color_camera_prim_path, domain_id, width, height):
    """建发布 action graph: /clock + color image + camera_info, 随 sim.step 自动 tick。

    所有节点挂在 OnPlaybackTick 下; image 和 camera_info 共用同一 render product +
    同一 OnPlaybackTick, 所以 timestamp 同帧, 满足 AprilTag 的 ExactTime 同步 (spec §4)。
    """
    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": PUB_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnTick.outputs:tick", "CreateRP.inputs:execIn"),
                ("CreateRP.outputs:execOut", "CameraRgb.inputs:execIn"),
                ("CreateRP.outputs:execOut", "CameraInfo.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "CameraRgb.inputs:context"),
                ("Context.outputs:context", "CameraInfo.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("CreateRP.outputs:renderProductPath", "CameraRgb.inputs:renderProductPath"),
                ("CreateRP.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(domain_id)),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("PublishClock.inputs:topicName", CLOCK_TOPIC),
                ("CreateRP.inputs:cameraPrim", [color_camera_prim_path]),
                ("CreateRP.inputs:width", int(width)),
                ("CreateRP.inputs:height", int(height)),
                ("CameraRgb.inputs:type", "rgb"),
                ("CameraRgb.inputs:topicName", COLOR_IMAGE_TOPIC),
                ("CameraRgb.inputs:frameId", COLOR_OPTICAL_FRAME),
                ("CameraInfo.inputs:topicName", COLOR_INFO_TOPIC),
                ("CameraInfo.inputs:frameId", COLOR_OPTICAL_FRAME),
            ],
        },
    )
    return PUB_GRAPH_PATH


def setup_tag_tf_subscriber(domain_id, marker_prim_path=DEFAULT_TAG_MARKER_PRIM):
    """建订阅 action graph: 收 /tf, 按 frameNamesMap 把 tag child frame 套到 marker prim。

    ROS2SubscribeTransformTree 不是把 TF 读进 Python, 而是用 GfTransform/UsdGeomXformable
    把收到的 TF 直接写进 prim 的变换 (spec §1)。所以回路通 = marker prim 移动。
    """
    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": SUB_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubTF", "isaacsim.ros2.bridge.ROS2SubscribeTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "SubTF.inputs:execIn"),
                ("Context.outputs:context", "SubTF.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(domain_id)),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("SubTF.inputs:frameNamesMap", build_tag_frame_names_map(marker_prim_path)),
            ],
        },
    )
    return SUB_GRAPH_PATH
