"""OmniGraph ROS2 wiring for the B2ARX simulation clock and Nav2 commands.

Isaac Sim 自带 jazzy ROS2 库, C++ plugin 直接用; 在 py3.11 的 isaaclab 环境里
import rclpy 会因 ABI 不匹配崩溃, 所以发布与订阅全部走 OmniGraph 节点。
omni.graph.core 只在 isaacsim 运行时存在, 所以建图函数里才 import, 模块顶层
保持纯逻辑可单测。
"""
from __future__ import annotations


CLOCK_GRAPH_PATH = "/World/B2ArxROS2ClockGraph"
CMD_VEL_GRAPH_PATH = "/World/B2ArxROS2CmdVelGraph"
XT32_POINTCLOUD_GRAPH_PATH = "/World/B2ArxXT32PointCloudGraph"
WRIST_CAMERA_GRAPH_PATH = "/World/B2ArxWristCameraGraph"

CLOCK_TOPIC = "/clock"
CMD_VEL_TOPIC = "/cmd_vel"
CMD_VEL_HEARTBEAT_TOPIC = "/cmd_vel_heartbeat"
# Match the real Hesai driver contract so simulation and hardware can feed the
# same downstream Nav2/Nvblox configuration without a remapping fork.
XT32_POINTCLOUD_TOPIC = "/lidar_points"
XT32_FRAME_ID = "hesai_lidar"

# Stable sim/real wrist-camera contract.  The simulation graph below uses the
# official Isaac Sim camera helpers; the real-robot launch uses Intel's
# realsense2_camera driver with depth alignment enabled.
WRIST_COLOR_TOPIC = "/wrist_camera/color/image_raw"
WRIST_COLOR_INFO_TOPIC = "/wrist_camera/color/camera_info"
WRIST_ALIGNED_DEPTH_TOPIC = "/wrist_camera/aligned_depth_to_color/image_raw"
WRIST_ALIGNED_DEPTH_INFO_TOPIC = "/wrist_camera/aligned_depth_to_color/camera_info"
WRIST_COLOR_OPTICAL_FRAME = "wrist_camera_color_optical_frame"
WRIST_SIM_DEPTH_ENCODING = "32FC1"
WRIST_SIM_DEPTH_SCALE = 1.0


def setup_ros2_clock(domain_id):
    """Publish one simulation clock shared by all ROS consumers."""
    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": CLOCK_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(domain_id)),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("PublishClock.inputs:topicName", CLOCK_TOPIC),
            ],
        },
    )
    return CLOCK_GRAPH_PATH


def setup_ros2_cmd_vel_subscriber(
    domain_id,
    topic_name=CMD_VEL_TOPIC,
    heartbeat_topic_name=CMD_VEL_HEARTBEAT_TOPIC,
):
    """Subscribe to Nav2 Twist commands and an explicit transport heartbeat.

    The Twist remains the unmodified navigation command. A separate UInt32
    sequence changes on every watchdog publication, so the policy can detect
    message arrival even when every consecutive Twist is the same zero value.
    """
    if not str(topic_name).startswith("/"):
        raise ValueError(f"cmd_vel topic must be absolute: {topic_name!r}")
    if not str(heartbeat_topic_name).startswith("/"):
        raise ValueError(f"cmd_vel heartbeat topic must be absolute: {heartbeat_topic_name!r}")

    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": CMD_VEL_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("SubscribeHeartbeat", "isaacsim.ros2.bridge.ROS2Subscriber"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "SubscribeHeartbeat.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("Context.outputs:context", "SubscribeHeartbeat.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(domain_id)),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("SubscribeTwist.inputs:topicName", str(topic_name)),
                ("SubscribeTwist.inputs:queueSize", 1),
                ("SubscribeHeartbeat.inputs:messagePackage", "std_msgs"),
                ("SubscribeHeartbeat.inputs:messageSubfolder", "msg"),
                ("SubscribeHeartbeat.inputs:messageName", "UInt32"),
                ("SubscribeHeartbeat.inputs:topicName", str(heartbeat_topic_name)),
                ("SubscribeHeartbeat.inputs:queueSize", 1),
            ],
        },
    )
    return CMD_VEL_GRAPH_PATH


def setup_ros2_xt32_pointcloud(
    domain_id,
    lidar_prim_path,
    topic_name=XT32_POINTCLOUD_TOPIC,
    frame_id=XT32_FRAME_ID,
    *,
    full_scan=True,
    show_debug_view=False,
):
    """Publish the embedded Hesai XT32 as ``sensor_msgs/PointCloud2``.

    The official XT32 USD contains an ``OmniLidar`` prim but its ROS variant is
    disabled in the core robot asset.  This graph creates the RTX render product
    and attaches Isaac Sim's supported ROS2 RTX Lidar writer without modifying
    or duplicating the sensor prim.
    """
    if not str(lidar_prim_path).startswith("/"):
        raise ValueError(f"XT32 lidar prim path must be absolute: {lidar_prim_path!r}")
    if not str(topic_name).startswith("/"):
        raise ValueError(f"XT32 point cloud topic must be absolute: {topic_name!r}")
    if not str(frame_id) or str(frame_id).startswith("/"):
        raise ValueError(f"XT32 frame id must be non-empty and have no leading slash: {frame_id!r}")

    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": XT32_POINTCLOUD_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishPointCloud", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),
                ("RunOneSimulationFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishPointCloud.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishPointCloud.inputs:renderProductPath",
                ),
                ("Context.outputs:context", "PublishPointCloud.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(domain_id)),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("CreateRenderProduct.inputs:cameraPrim", [str(lidar_prim_path)]),
                ("CreateRenderProduct.inputs:width", 128),
                ("CreateRenderProduct.inputs:height", 128),
                ("PublishPointCloud.inputs:topicName", str(topic_name)),
                ("PublishPointCloud.inputs:frameId", str(frame_id)),
                ("PublishPointCloud.inputs:type", "point_cloud"),
                ("PublishPointCloud.inputs:queueSize", 1),
                ("PublishPointCloud.inputs:fullScan", bool(full_scan)),
                ("PublishPointCloud.inputs:showDebugView", bool(show_debug_view)),
                ("PublishPointCloud.inputs:useSystemTime", False),
                # Keep the RTX writer on the same monotonic simulation-time
                # contract as /clock and the official ZED helper.  Resetting
                # only the lidar timestamp on a UI Stop/Play cycle makes its
                # messages older than /clock and causes TF/Nvblox to reject
                # otherwise valid scans.
                ("PublishPointCloud.inputs:resetSimulationTimeOnStop", False),
            ],
        },
    )
    return XT32_POINTCLOUD_GRAPH_PATH


def setup_ros2_wrist_camera(
    domain_id,
    camera_prim_path,
    *,
    width=1280,
    height=720,
    frame_skip_count=0,
    frame_id=WRIST_COLOR_OPTICAL_FRAME,
    color_topic=WRIST_COLOR_TOPIC,
    color_info_topic=WRIST_COLOR_INFO_TOPIC,
    aligned_depth_topic=WRIST_ALIGNED_DEPTH_TOPIC,
    aligned_depth_info_topic=WRIST_ALIGNED_DEPTH_INFO_TOPIC,
):
    """Publish aligned wrist RGB-D from one official Isaac Sim camera.

    Both the ``rgb`` and ``depth`` Camera Helpers intentionally consume the
    same render product.  The embedded official D455 asset also contains a
    ``Camera_Pseudo_Depth`` prim, but that prim is offset from the color camera
    by about 11.5 mm.  Publishing it under an ``aligned_depth_to_color`` name
    would therefore be false registration and recreates the old view mismatch.

    Isaac Sim's official depth writer publishes ``32FC1`` in metres.  A real
    RealSense generally publishes ``16UC1`` in millimetres, so downstream
    algorithms must select their depth scale by sensor mode even though topic
    names are stable.
    """
    if not str(camera_prim_path).startswith("/"):
        raise ValueError(
            f"wrist camera prim path must be absolute: {camera_prim_path!r}"
        )
    topics = {
        "color topic": color_topic,
        "color camera-info topic": color_info_topic,
        "aligned-depth topic": aligned_depth_topic,
        "aligned-depth camera-info topic": aligned_depth_info_topic,
    }
    for label, topic in topics.items():
        if not str(topic).startswith("/"):
            raise ValueError(f"wrist {label} must be absolute: {topic!r}")
    if len(set(str(topic) for topic in topics.values())) != len(topics):
        raise ValueError("wrist camera topics must be unique")
    if not str(frame_id) or str(frame_id).startswith("/"):
        raise ValueError(
            f"wrist camera frame id must be non-empty and have no leading slash: {frame_id!r}"
        )
    if isinstance(width, bool) or int(width) <= 0:
        raise ValueError(f"wrist camera width must be positive: {width!r}")
    if isinstance(height, bool) or int(height) <= 0:
        raise ValueError(f"wrist camera height must be positive: {height!r}")
    if isinstance(frame_skip_count, bool) or int(frame_skip_count) < 0:
        raise ValueError(
            "wrist camera frame_skip_count must be a non-negative integer: "
            f"{frame_skip_count!r}"
        )

    import omni.graph.core as og

    publishers = (
        "PublishColor",
        "PublishDepth",
        "PublishColorInfo",
        "PublishDepthInfo",
    )
    connections = [
        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
    ]
    for publisher in publishers:
        connections.extend(
            [
                ("CreateRenderProduct.outputs:execOut", f"{publisher}.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    f"{publisher}.inputs:renderProductPath",
                ),
                ("Context.outputs:context", f"{publisher}.inputs:context"),
                ("SensorDataQoS.outputs:qosProfile", f"{publisher}.inputs:qosProfile"),
            ]
        )

    common_values = []
    for publisher in publishers:
        common_values.extend(
            [
                (f"{publisher}.inputs:frameId", str(frame_id)),
                (f"{publisher}.inputs:frameSkipCount", int(frame_skip_count)),
                (f"{publisher}.inputs:queueSize", 5),
                (f"{publisher}.inputs:useSystemTime", False),
                (f"{publisher}.inputs:resetSimulationTimeOnStop", False),
            ]
        )

    og.Controller.edit(
        {"graph_path": WRIST_CAMERA_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SensorDataQoS", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishColor", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishColorInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("PublishDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: connections,
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(domain_id)),
                ("Context.inputs:useDomainIDEnvVar", False),
                ("SensorDataQoS.inputs:history", "keepLast"),
                ("SensorDataQoS.inputs:depth", 5),
                ("SensorDataQoS.inputs:reliability", "bestEffort"),
                ("SensorDataQoS.inputs:durability", "volatile"),
                ("SensorDataQoS.inputs:liveliness", "systemDefault"),
                ("CreateRenderProduct.inputs:cameraPrim", [str(camera_prim_path)]),
                ("CreateRenderProduct.inputs:width", int(width)),
                ("CreateRenderProduct.inputs:height", int(height)),
                ("PublishColor.inputs:topicName", str(color_topic)),
                ("PublishColor.inputs:type", "rgb"),
                ("PublishDepth.inputs:topicName", str(aligned_depth_topic)),
                ("PublishDepth.inputs:type", "depth"),
                ("PublishColorInfo.inputs:topicName", str(color_info_topic)),
                ("PublishDepthInfo.inputs:topicName", str(aligned_depth_info_topic)),
                *common_values,
            ],
        },
    )
    return WRIST_CAMERA_GRAPH_PATH
