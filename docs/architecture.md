# B2ARX 系统架构

本仓库是 B2ARX 的资产、配置、适配器和启动编排层，不维护 ZED、Hesai、Nvblox、Nav2 或 Isaac ROS 算法的私有分叉。

## 分层

```text
仿真输入                                  真机输入
my_robot.usd + Isaac Sim                  ZED X + Hesai XT32 + D435i + 机器人驱动
  ├─ ZED SDK simulation stream              ├─ zed_wrapper hardware mode
  ├─ /lidar_points                          ├─ /lidar_points
  ├─ /wrist_camera/*                        ├─ /wrist_camera/*
  └─ articulation state                     └─ /b2/odom + 机器人状态
           │                                      │
           └──────────── sensor/robot adapter ────┘
                                  │
                                  ▼
                      稳定的 B2ARX ROS 接口
                                  │
                 ┌────────────────┼────────────────┐
                 ▼                ▼                ▼
            Isaac ROS Nvblox     Nav2        其它 Isaac ROS 算法
                 │                │
                 └──── map/costmap┘
                                  │ /cmd_vel
                                  ▼
                         B2 locomotion adapter
```

仿真和真机只替换输入适配器。Nvblox、Nav2、感知和操作算法应继续 include 上游 launch，并加载本项目的参数覆盖与 remap。

## 运行容器边界

- ZED 使用官方 `zed_camera.launch.py`，由上游 launch 管理独立的 ZED 容器和 SDK 生命周期。
- 默认 Nvblox 使用官方 `nvblox.launch.py` 创建唯一共享容器，composed Nav2 加载到该容器。
- 当前稳定 profile 设置 `disable_zed_nitros:=true`，ZED 到 Nvblox 走标准
  `sensor_msgs/Image`；Managed NITROS 只保留为显式 A/B 开关。

这意味着“复用官方 launch”不等于把所有组件强塞进同一个容器。容器所有权仍由对应上游
入口管理，本项目只传参数、选择 adapter 和组合生命周期。

Isaac ROS 4.5 的 Nvblox + composed Nav2 共享容器在 Ctrl-C 后，可能于组件析构阶段以
`-11` 结束。这属于当前上游 shutdown 路径的已知行为。它只适用于“SIGINT 之后的退出期”判断；
运行过程中自行出现 `-11` 仍是故障，不能用该已知行为解释。运行成功与否应以关停前的 lifecycle、
Nvblox map slice、Nav2 costmap 和数据流验收结果为准。

## 上游复用边界

| 子系统 | 复用入口 | 本项目只负责 |
| --- | --- | --- |
| ZED | `zed_wrapper/launch/zed_camera.launch.py` | sim/real 模式、地址/端口或序列号、参数覆盖 |
| Nvblox | `nvblox_examples_bringup/launch/perception/nvblox.launch.py` | B2ARX 参数覆盖、容器编排 |
| Nav2 | `nav2_bringup/launch/navigation_launch.py` | B2 footprint、costmap 参数、速度安全适配 |
| Hesai XT32 | `hesai_ros_driver_node` | 传入官方驱动配置文件，统一 topic/frame |
| RealSense D435i | `realsense2_camera/launch/rs_launch.py` | 统一腕部话题、开关和外参 frame |
| ARX/操作算法 | 本机 Isaac ROS manipulation 与 ARX bringup launch | 机器人、相机接口和 TF remap |

## 稳定接口合同

### ZED X

```text
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/camera_info
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/camera_info
/zed/zed_node/pose
/zed/zed_node/odom
```

仿真和真机使用同一个官方 `zed_wrapper`。仿真启用 SDK simulation stream；真机关闭 simulation mode 并选择真实设备。

### XT32

```text
topic: /lidar_points
type:  sensor_msgs/msg/PointCloud2
frame: hesai_lidar
rate:  10 Hz nominal
```

仿真点云目前保证 `x/y/z` 字段。真实 Hesai 驱动还会提供 `intensity/ring/timestamp`；依赖这些字段的算法需要单独的仿真字段适配，不能假设两端字段完全相同。

### 腕部 RGB-D

```text
/wrist_camera/color/image_raw
/wrist_camera/color/camera_info
/wrist_camera/aligned_depth_to_color/image_raw
/wrist_camera/aligned_depth_to_color/camera_info
frame: wrist_camera_color_optical_frame
```

Isaac Sim 5.1 没有官方 D435i USD，因此仿真端以官方 D455 Color camera 作为接口代理，RGB 和 renderer depth 共用一个 render product；真机端由官方 RealSense 驱动执行硬件时间戳、同步和 depth-to-color alignment。topic/frame 合同一致不代表成像模型一致。仿真 depth 是 `32FC1` 米，真机通常是 `16UC1` 毫米，下游必须使用对应 scale。

### 导航和控制

```text
/b2/odom       nav_msgs/msg/Odometry，child_frame_id=base_link
/cmd_vel       geometry_msgs/msg/Twist，唯一发布者为项目 watchdog
/cmd_vel_heartbeat
```

ZED 的原始 odometry 以相机为 child frame。B2ARX odometry adapter 负责固定外参和杆臂速度修正，算法层只使用 `/b2/odom`。真机若已有底盘里程计，也应直接适配到 `/b2/odom`。

ZED mount、XT32 mount、frame 和 odometry adapter IO 集中在
`config/platform_adapters.yaml`。`platform_adapters.launch.py` 从同一个 ZED mount 同时生成
静态 TF 和 odometry 杆臂参数，避免 sim/real 或 TF/odom 各自维护一份外参。

## 配置所有权

| 模块 | 项目配置入口 |
| --- | --- |
| Isaac Sim 资产/环境/传感器开关 | `config/simulation/*.yaml` |
| locomotion policy | `config/policies/*.yaml` |
| ZED wrapper 覆盖 | `config/zedx_nvblox_release_4_5.yaml` |
| Nvblox 覆盖 | `config/nvblox_b2arx.yaml` |
| Nav2 与速度安全节点 | `config/b2arx_nav2.yaml` |
| B2ARX TF/odom/XT32 安装 frame | `config/platform_adapters.yaml` |
| D435i 真机 stream | `config/wrist_realsense_d435i.yaml` |
| easy_handeye 原始结果 | `config/arx_r5a_d543if_eih.calib` |
| manipulation 腕部外参 | `config/wrist_d435i_eye_in_hand.yaml` |
| 真机 Hesai packet/calibration | `HESAI_CONFIG_FILE` 指向的官方驱动 YAML |

## TF 所有权

当前 ZED 定位模式使用：

```text
map -> odom -> zed_camera_link -> base_link -> hesai_lidar
```

- `map -> odom -> zed_camera_link`：官方 ZED wrapper。
- `zed_camera_link -> base_link`：B2ARX ZED 安装外参。
- `base_link -> hesai_lidar`：机器人 URDF 或 B2ARX 平台适配器，二者只能启用一个。
- `base_link -> ... -> link6`：机器人 description / `robot_state_publisher`，随关节运动。
- `link6 -> wrist_camera_link`：easy_handeye eye-in-hand 测量；不能误写为静态 `base_link -> camera`，也不能求逆。
- `wrist_camera_link -> wrist_camera_color_optical_frame`：官方 RealSense 驱动内部 TF。
- 相机内部 optical frames：对应相机驱动或官方描述包。

真机若由机器人定位系统拥有 `map/odom/base_link`，必须关闭 ZED 的重复 TF 发布，并只保留传感器安装外参。

## 策略进程边界

当前 locomotion policy 以 200 Hz 直接读取和写入 Isaac articulation，因此仿真策略执行器仍与 Isaac Sim 同进程。策略模型、部署参数和输入接口已经独立配置；把执行器拆到另一个终端还需要关节状态、IMU、根状态、关节命令、时间同步和 watchdog 的完整 ROS 控制接口，不能通过目录重排替代。

## 腕部 D435i 边界

腕部 sim/real ROS 接口已经拉通。仿真绑定核心 USD 中的官方 D455 Color prim，真机 include 官方 D435i 驱动；操作 profile 继续 include 现有 ARX Isaac ROS manipulation launch。

真实 easy_handeye 文件保存的是 `link6 -> D543if_link`。项目转换器只把旧 frame 名显式映射为 `wrist_camera_link`，不改变平移、四元数或方向。操作栈在图像时间使用完整动态链：

```text
base_link -> ... -> link6 -> wrist_camera_link -> wrist_camera_color_optical_frame
```

`GetObjectPose` 本身没有 Header，所以腕部 profile 会复用上游 launch 的 exact-time TF 转换，并把交给行为树的 pose frame 改写成 `base_link`。真实外参不能无条件覆盖 authored USD；两者只有在物理安装被明确做成相同时才能共用数值。
