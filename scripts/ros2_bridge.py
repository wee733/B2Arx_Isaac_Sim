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

# parent frame (d455_color_optical_frame) 对应的 prim。AprilTag 的 /tf 是 tag 相对这个
# frame 的位姿, SubscribeTransformTree 需要 parent 在 frameNamesMap 里, 用 parent prim 的
# 世界变换把 tag 位姿换算到世界坐标写进 marker (官方 test_subscribers.py 的 map 含 parent+child)。
#
# 关键: 不能直接用 color 相机 prim。USD 相机是 opengl 约定 (-Z 前/+Y 上), 而 AprilTag 的 TF
# 是 ROS 光学约定 (+Z 前/-Y 上), 两者差「绕 X 轴 180°」(IsaacLab math.py 官方公式
# T_ROS = diag(1,-1,-1)·T_USD)。直接用相机 prim 会把 tag 的"前方"当成"后方", marker 落到
# 机器人身上。所以 parent 映射到相机 prim 下一个额外绕 X 转 180° 的子 prim (ROS 光学系)。
COLOR_OPTICAL_FRAME_SUBPRIM = "ros_optical"
DEFAULT_COLOR_CAMERA_PRIM = (
    "/World/envs/env_0/Robot/R5a_link6/D455/RSD455/Camera_OmniVision_OV9782_Color"
)
DEFAULT_COLOR_OPTICAL_PRIM = f"{DEFAULT_COLOR_CAMERA_PRIM}/{COLOR_OPTICAL_FRAME_SUBPRIM}"

# USD/opengl -> ROS 光学系: 绕 X 轴 180°, 四元数 wxyz (旋转矩阵 = diag(1,-1,-1))。
ROS_OPTICAL_QUAT_WXYZ = (0.0, 1.0, 0.0, 0.0)


def build_tag_frame_names_map(marker_prim_path: str, color_optical_prim_path: str) -> list[str]:
    """构造 ROS2SubscribeTransformTree.frameNamesMap。

    顺序是 [prim_path, frame_id, ...] (偶数长度)。本机验证来源:
    docs/ogn/OgnROS2SubscribeTransformTree.rst:41 ("[prim_path_0, frame_name_0, ...]")
    + tests/test_subscribers.py:555 (["/World","world","/World/cube","cube"])。

    **parent + child 都要映射**: AprilTag 的 /tf 里 frame_id=d455_color_optical_frame
    (parent)、child_frame_id=tag36h11:0。官方测试的 map 同时给了 parent("/World"↔"world")
    和 child("/World/cube"↔"cube")。只给 child 会让节点找不到 parent 参考系, marker 不动
    (spec R1 验证: parent 必须在 map 里)。顺序写反 (frame_id 在前) 同样会断链。

    parent 映射到 ROS 光学系子 prim (setup_color_optical_frame_prim 建的, 绕 X 转 180°),
    不是相机 prim 本身, 否则 ROS/USD 朝向差 180° 会让 marker 落到机器人身上。
    """
    return [
        color_optical_prim_path, COLOR_OPTICAL_FRAME,  # parent: ROS 光学系子 prim
        marker_prim_path, TAG_TF_CHILD_FRAME,          # child: tag
    ]


def setup_color_optical_frame_prim(color_camera_prim_path=DEFAULT_COLOR_CAMERA_PRIM):
    """在 color 相机 prim 下建一个绕 X 转 180° 的子 Xform = ROS 光学系 parent frame。

    USD 相机是 opengl 约定 (-Z 前), AprilTag TF 是 ROS 约定 (+Z 前), 差绕 X 180°
    (IsaacLab math.py: T_ROS = diag(1,-1,-1)·T_USD)。SubscribeTransformTree 用这个子 prim
    的世界变换当 parent 参考系, tag 的 +Z 前方才会正确落到相机前方的桌面, 而非身后。
    返回子 prim 路径, 供 frameNamesMap 用。
    """
    import omni.usd
    from pxr import Gf, UsdGeom

    stage = omni.usd.get_context().get_stage()
    optical_path = f"{color_camera_prim_path}/{COLOR_OPTICAL_FRAME_SUBPRIM}"
    xform = UsdGeom.Xform.Define(stage, optical_path)
    w, x, y, z = ROS_OPTICAL_QUAT_WXYZ
    xform.AddOrientOp().Set(Gf.Quatf(float(w), float(x), float(y), float(z)))
    return optical_path


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


def setup_tag_tf_subscriber(
    domain_id,
    marker_prim_path=DEFAULT_TAG_MARKER_PRIM,
    color_optical_prim_path=DEFAULT_COLOR_OPTICAL_PRIM,
):
    """建订阅 action graph: 收 /tf, 按 frameNamesMap 把 tag child frame 套到 marker prim。

    ROS2SubscribeTransformTree 不是把 TF 读进 Python, 而是用 GfTransform/UsdGeomXformable
    把收到的 TF 直接写进 prim 的变换 (spec §1)。frameNamesMap 同时映射 parent(ROS 光学系
    子 prim) 和 child(tag), 否则节点找不到 parent 参考系, marker 不动。回路通 = marker 移动。
    注意: color_optical_prim_path 必须是 setup_color_optical_frame_prim 建的那个绕 X 转 180°
    的子 prim, 不是相机 prim 本身。
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
                (
                    "SubTF.inputs:frameNamesMap",
                    build_tag_frame_names_map(marker_prim_path, color_camera_prim_path),
                ),
            ],
        },
    )
    return SUB_GRAPH_PATH
