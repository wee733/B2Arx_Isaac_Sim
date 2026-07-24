# Isaac ROS 官方/开源实现流程调研

本文记录对 Isaac ROS 官方仓库、Isaac Sim ROS2 Bridge 示例和相关 launch 代码的阅读结果。目标是为 B2ARX 项目设计 Thor 算法仓库和 Isaac Sim 执行端 ROS2 接线方式。

> 当前运行事实（2026-07-24）：本文主体是选型调研和后续算法仓库建议，最终运行入口以
> [README](../README.md) 和 [系统架构](architecture.md) 为准。当前 ZED 由官方
> `zed_camera.launch.py` 管理独立的 ZED 容器；官方 Nvblox launch 创建默认共享容器，
> composed Nav2 加载到该容器。经当前 RTX 5090/CUDA 13 主机验证，默认
> `disable_zed_nitros:=true`，ZED 到 Nvblox 使用稳定的 `sensor_msgs/Image` 路径，
> 并不是旧调研阶段设想的“ZED/Nvblox 同容器 Managed NITROS”拓扑。

## 1. 调研结论

官方实现的核心思想不是“每个算法自己处理所有相机差异”，而是：

```text
数据源 fragment
  -> 把 ZED/rosbag/Isaac Sim 输入标准化
  -> 输出统一算法接口: image_rect, camera_info_rect, depth, left/right image

算法 fragment
  -> 只认标准接口
  -> 用 ComposableNode 拼成 GPU pipeline
  -> 输出标准 ROS2 结果: Detection2DArray、AprilTagDetectionArray、TF、Pose

interface_specs_file
  -> 覆盖相机分辨率、frame、内参近似值、模型配置等 launch-time 参数
```

这对本项目的直接启发：

- `b2arx_isaac_sim` 负责提供官方 ZED SDK simulation stream，标准 ROS2 传感器 topic 由
  `zed_wrapper` 发布。
- Thor 端应该有独立仓库，模仿 `isaac_ros_examples`，按 fragment/launch/config 组织算法。
- Thor 端不要在每个算法里重复写死 ZED topic，而应先用输入 fragment 把 wrapper topic remap
  成算法统一接口。
- 当前 Nvblox 仍直接复用官方 Isaac ROS launch，但默认消费 ZED 的标准 ROS 图像 topic；
  `disable_zed_nitros:=false` 只作为升级驱动/SDK 后的显式 A/B 选项。

## 2. 官方仓库和关键文件

### 2.1 `isaac_ros_examples`

仓库：

```text
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_examples
```

关键文件：

```text
isaac_ros_examples/launch/isaac_ros_examples.launch.py
isaac_ros_examples/isaac_ros_examples/isaac_ros_launch_fragment.py
isaac_ros_examples/isaac_ros_examples/isaac_ros_launch_fragment_spec.py
```

核心流程：

```text
用户传入:
  launch_fragments:=<data_source>,apriltag
  interface_specs_file:=xxx.json

isaac_ros_examples.launch.py:
  1. 解析 launch_fragments 字符串
  2. 查 LAUNCH_FRAGMENT_SPECS
  3. 动态 import 每个 fragment class
  4. 收集所有 fragment.get_interface_specs()
  5. 用 interface_specs_file 覆盖默认 specs
  6. 调每个 fragment.get_composable_nodes(interface_specs)
  7. 放进一个 component_container_mt
```

值得抄的点：

- launch fragment 是最干净的扩展边界。
- 每个 fragment 提供：
  - `get_interface_specs()`
  - `get_composable_nodes(interface_specs)`
  - `get_launch_actions(interface_specs)`
- 数据源和算法是可组合的。
- `interface_specs_file` 是配置覆盖入口，不需要为每个相机写一堆 CLI 参数。

对我们 Thor 仓库的建议：

```text
b2arx_thor_ros/
  b2arx_thor_examples/
    launch/b2arx_thor_examples.launch.py
    b2arx_thor_examples/launch_fragment.py
    b2arx_thor_examples/launch_fragment_spec.py
  b2arx_zed_input/
    launch/zed_mono_rect.launch.py
    launch/zed_mono_rect_depth.launch.py
  b2arx_apriltag/
    config/apriltag_sim_interface_specs.json
  b2arx_foundationpose/
    config/foundationpose_sim_interface_specs.json
```

## 3. ZED 输入标准化

`zed_wrapper` 已提供 rectified RGB、CameraInfo 和 registered depth。算法 fragment 不应重新实现
SDK stream 接收，而只负责把现有 topic 映射成统一接口：

```text
/zed/zed_node/rgb/color/rect/image  -> image_rect
/zed/zed_node/rgb/camera_info      -> camera_info_rect
```

RGBD 算法再增加：

```text
/zed/zed_node/depth/depth_registered -> depth
/zed/zed_node/depth/camera_info       -> depth_camera_info
```

深度 encoding 和单位必须以运行时消息及 wrapper 文档为准。当前 Nvblox 合同使用注册深度，不能仅靠
修改 topic 名或 `frame_id` 来伪造对齐关系。默认配置关闭 ZED Managed NITROS，因此验收标准是
`sensor_msgs/Image` publisher/subscriber 和实际消息；只有显式设置
`disable_zed_nitros:=false` 做 A/B 时，才额外检查 NITROS 协商结果。

## 4. AprilTag 官方实现

仓库：

```text
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
```

关键文件：

```text
isaac_ros_apriltag/launch/isaac_ros_apriltag_core.launch.py
isaac_ros_apriltag/launch/isaac_ros_apriltag_isaac_sim_pipeline.launch.py
isaac_ros_apriltag/src/apriltag_node.cpp
isaac_ros_apriltag/include/isaac_ros_apriltag/apriltag_node.hpp
```

### 4.1 Launch 层

`isaac_ros_apriltag_core.launch.py` 中：

```python
ComposableNode(
  package='isaac_ros_apriltag',
  plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
  name='apriltag',
  parameters=[{
    'size': 0.22,
    'max_tags': 64,
    'tile_size': 4,
    'tag_family': tag_family,
    'backends': backends
  }],
  remappings=[
    ('image', 'image_rect'),
    ('camera_info', 'camera_info_rect')
  ]
)
```

说明：

- AprilTag 节点内部订阅的是 `image` 和 `camera_info`。
- 官方 fragment 把它 remap 到统一接口 `image_rect`、`camera_info_rect`。
- 默认后端是 CUDA。
- tag size 默认 0.22 m，要按真实 tag 尺寸改。

Isaac Sim 专用 pipeline 中：

```python
remappings=[
  ('image', 'front_stereo_camera/left/image_rect_color'),
  ('camera_info', 'front_stereo_camera/left/camera_info')
]
```

说明官方对 Isaac Sim 的处理也很直接：Sim 发布相机 topic，AprilTag launch remap 过去。

### 4.2 C++ 节点层

`apriltag_node.cpp` 里关键行为：

- 参数：

```text
max_tags
size
tile_size
tag_family
backends
```

- 同步策略：

```cpp
message_filters::sync_policies::ExactTime<NitrosImage, CameraInfo>
```

说明：图像和 CameraInfo 必须同 timestamp，否则可能不同步。

- 订阅：

```cpp
image_sub_.subscribe(this, "image");
camera_info_sub_.subscribe(this, "camera_info");
```

- 输出：

```text
tag_detections
tf
```

- 内参：

```cpp
fx = camera_info->k[0]
fy = camera_info->k[4]
cx = camera_info->k[2]
cy = camera_info->k[5]
```

说明：CameraInfo.K 矩阵会直接决定 pose 准不准。

- 编码支持：

VPI 路径支持：

```text
rgb8
bgr8
rgba8
bgra8
mono8
```

cuAprilTags 路径只支持：

```text
rgb8
bgr8
```

第一版建议 Sim 发布 `rgb8`，避免踩 encoding 坑。

### 4.3 对本项目的要求

Sim 侧必须做到：

```text
Image:
  encoding = rgb8
  header.stamp == CameraInfo.header.stamp
  header.frame_id == CameraInfo.header.frame_id

CameraInfo:
  width/height 与实际 ZED stream 一致
  K/P/R 填对
  header.frame_id 与图像一致
```

Thor 侧 fragment：

```text
/zed/zed_node/rgb/color/rect/image
  -> image_rect

/zed/zed_node/rgb/camera_info
  -> camera_info_rect
```

然后直接复用官方：

```bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
  launch_fragments:=apriltag \
  interface_specs_file:=...
```

或自建：

```bash
ros2 launch b2arx_thor_examples b2arx_thor_examples.launch.py \
  launch_fragments:=zed_mono_rect,apriltag
```

## 5. RT-DETR 官方实现

仓库：

```text
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection
```

关键文件：

```text
isaac_ros_rtdetr/launch/isaac_ros_rtdetr_core.launch.py
isaac_ros_rtdetr/launch/isaac_ros_rtdetr.launch.py
```

官方流程：

```text
image_rect + camera_info_rect
  -> ResizeNode
  -> PadNode
  -> ImageFormatConverterNode(rgb8)
  -> ImageToTensorNode
  -> InterleavedToPlanarNode
  -> ReshapeNode
  -> RtDetrPreprocessorNode
  -> TensorRTNode
  -> RtDetrDecoderNode
  -> detections_output
```

输出：

```text
detections_output
  type: vision_msgs/msg/Detection2DArray
```

关键点：

- RT-DETR 模型输入固定 640x640。
- fragment 使用 `interface_specs['camera_resolution']` 决定原始图像大小。
- `ImageFormatConverterNode` 明确转成 `rgb8`。
- TensorRT engine 路径通过 launch 参数传入。

对本项目：

- Thor 仓库应把 RT-DETR 作为第二阶段。
- 输入继续复用 `zed_mono_rect`。
- 输出 `detections_output` 后，可以：
  - 直接可视化；
  - 作为 FoundationPose 的 bbox 输入；
  - 或自己写 selector，把目标类别/置信度转成 `/b2arx/target_hint`。

## 6. FoundationPose 官方实现

仓库：

```text
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation
```

关键文件：

```text
isaac_ros_foundationpose/launch/isaac_ros_foundationpose_core.launch.py
isaac_ros_foundationpose/launch/isaac_ros_foundationpose.launch.py
```

官方 core pipeline：

```text
image_rect + camera_info_rect + depth
  -> NitrosCameraDropNode
  -> rgb/image_rect_color
  -> rgb/camera_info
  -> depth_image

rgb/image_rect_color
  -> RT-DETR preprocess
  -> TensorRT
  -> RtDetrDecoder
  -> detections_output

detections_output
  -> Detection2DArrayFilter
  -> Detection2DToMask
  -> segmentation

rgb/image_rect_color + depth_image + rgb/camera_info + segmentation
  -> FoundationPoseNode
  -> output
```

FoundationPose 输入：

```text
pose_estimation/depth_image
pose_estimation/image
pose_estimation/camera_info
pose_estimation/segmentation
```

FoundationPose 输出：

```text
pose_estimation/output
pose_estimation/pose_matrix_output
```

重要要求：

- 需要目标物体 mesh。
- 官方文档强调 mesh origin 最好在物体中心，否则 pose 会偏。
- 首次 pose estimation 计算重，tracking 会快一些。
- RGB、depth、segmentation 必须尺寸一致。

对本项目：

- 如果场景里物体 mesh 已知，FoundationPose 是“仿真识别 -> 真实抓取”非常合理的第二/三阶段。
- 但第一步仍然不该直接上 FoundationPose。先 AprilTag 验证相机/CameraInfo/TF，再 RT-DETR 验证检测，再 FoundationPose 验证 RGBD/mesh/pose。

## 7. NITROS Bridge 官方实现

仓库：

```text
https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
```

关键文件：

```text
isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_ros2/launch/isaac_ros_nitros_bridge_image_converter.launch.py
isaac_ros_nitros_bridge/isaac_ros_nitros_bridge_ros2/src/image_converter_node.cpp
isaac_ros_nitros_bridge/config/nitros_bridge_image_converter.yaml
```

官方流程：

```text
Isaac Sim nitros_bridge image topic
  -> ros2_input_bridge_image
  -> ImageConverterNode
  -> ros2_output_image / NITROS image topic
```

或反向：

```text
NITROS image topic
  -> ros2_input_image
  -> ImageConverterNode
  -> ros2_output_bridge_image
```

实现细节：

- `ImageConverterNode` 使用 CUDA IPC / fd / GPU buffer。
- launch 参数只暴露：

```text
pub_image_name
sub_image_name
```

判断：

- 这不是第一版要做的东西。
- 只有当标准 ROS2 图像传输性能不够，或者 Thor 上多个 NITROS 节点之间需要低拷贝时再上。
- 第一版先让标准 `sensor_msgs/Image` 跑通，Isaac ROS NITROS 节点会自己做 type negotiation。

## 8. Isaac Sim ROS2 Bridge 官方实现

官方文档：

```text
https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera.html
https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_python.html
```

本机 Isaac Sim extension 源码位置：

```text
/home/lbz/miniforge3/envs/isaaclab/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/
```

关键文件：

```text
isaacsim/ros2/bridge/impl/camera_info_utils.py
isaacsim/ros2/bridge/tests/test_camera.py
isaacsim/ros2/bridge/tests/test_camera_info.py
```

官方 Camera Helper 流程：

```text
OnPlaybackTick
  -> IsaacCreateRenderProduct(cameraPrim, width, height)
  -> ROS2CameraHelper(type=rgb/depth/depth_pcl/...)
  -> ROS2 topic

CameraInfo:
OnPlaybackTick
  -> IsaacCreateRenderProduct
  -> ROS2CameraInfoHelper
  -> sensor_msgs/CameraInfo
```

`camera_info_utils.py` 里计算 CameraInfo 的方式：

```text
fx = width * focalLength / horizontalAperture
fy = height * focalLength / verticalAperture
cx = width * 0.5
cy = height * 0.5
K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
R = identity
P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
```

如果相机 USD 有 OpenCV pinhole/fisheye distortion，会优先读 USD 里的 lens distortion 属性。

对本项目：

- 如果用 OmniGraph ROS2 Bridge，可以直接用 `ROS2CameraHelper` + `ROS2CameraInfoHelper`，最贴官方。
- 如果用 Python/rclpy 自己发图像，也要按这个规则填 CameraInfo。
- AprilTag 特别依赖 `CameraInfo.K`，不能随便填近似值。

## 9. 建议我们怎么抄

### 9.1 Thor 仓库结构

建议新建：

```text
b2arx_thor_ros/
  README.md
  b2arx_thor_examples/
    package.xml
    setup.py
    launch/b2arx_thor_examples.launch.py
    b2arx_thor_examples/launch_fragment.py
    b2arx_thor_examples/launch_fragment_spec.py

  b2arx_zed_input/
    package.xml
    setup.py
    launch/zed_mono_rect.launch.py
    launch/zed_mono_rect_depth.launch.py
    config/zed_interface_specs.json

  b2arx_perception/
    package.xml
    b2arx_perception/target_selector.py
    launch/apriltag_to_target.launch.py
    launch/detection_to_target.launch.py

  configs/
    apriltag_sim_interface_specs.json
    rtdetr_sim_interface_specs.json
    foundationpose_sim_interface_specs.json
```

### 9.2 第一版 launch fragment

`zed_mono_rect`：

```text
输入:
  /zed/zed_node/rgb/color/rect/image
  /zed/zed_node/rgb/camera_info

输出统一接口:
  image_rect
  camera_info_rect
```

`zed_mono_rect_depth`：

```text
输入:
  /zed/zed_node/rgb/color/rect/image
  /zed/zed_node/rgb/camera_info
  /zed/zed_node/depth/depth_registered

输出统一接口:
  image_rect
  camera_info_rect
  depth
```

`apriltag`：

直接复用官方 `isaac_ros_apriltag` fragment，或者 copy 一份轻量 launch。

### 9.3 调研阶段设想命令（历史）

Thor：

```bash
isaac-ros activate
export ROS_DOMAIN_ID=23
ros2 launch b2arx_thor_examples b2arx_thor_examples.launch.py \
  launch_fragments:=zed_mono_rect,apriltag \
  tag_family:=tag36h11 \
  tag_size:=0.22
```

Sim 主机：

```bash
conda activate isaaclab
export ROS_DOMAIN_ID=23
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --ros2 --print_zed_debug
```

上述命令保留为调研记录，不再是当前启动入口。现有仿真和导航分别使用
`scripts/run_isaac_sim.sh` 与 `scripts/run_isaac_ros.sh sim`；具体多终端命令见 README。

## 10. 对当前项目的设计决定

明确采用：

```text
标准 ROS2 topics first
官方 fragment 模式
Thor 单独仓库
Sim 端拥有资产、传感器和当前 articulation 策略执行器；ROS 算法继续复用官方 launch
```

暂不采用：

```text
第一版上 NITROS Bridge
第一版直接 FoundationPose 抓取
每个算法 launch 里重复写死 ZED topic
把 Thor 算法代码塞回 b2arx_isaac_sim
```

最小闭环顺序：

```text
1. Sim 发布 RGB + CameraInfo
2. Thor AprilTag 输出 /tag_detections
3. Sim 订阅 /tag_detections，只打印
4. Sim 转 target_world_to_sphere，只打印
5. 再进入策略闭环
6. 再上 RT-DETR
7. 再上 FoundationPose/RGBD
```

## 11. 官方参考

- Isaac ROS Examples：  
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_examples

- Isaac ROS AprilTag：  
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag

- Isaac ROS RT-DETR：  
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection/tree/release-4.4/isaac_ros_rtdetr

- Isaac ROS FoundationPose：  
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation/tree/release-4.4/isaac_ros_foundationpose

- Isaac ROS NITROS Bridge：  
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/tree/release-4.4/isaac_ros_nitros_bridge

- Isaac Sim ROS2 Cameras：  
  https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera.html

- Isaac Sim ROS2 Bridge Standalone Workflow：  
  https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_python.html
