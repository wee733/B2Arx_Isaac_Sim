# B2ARX Isaac Sim / Isaac ROS Bringup

这是 B2 机器狗 + ARX R5 机械臂的统一仿真、真机传感器和 Isaac ROS 算法编排仓库。

项目定位不是复制 ZED、Hesai、Nvblox、Nav2 或 Isaac ROS 的实现，而是维护：

- 核心 B2ARX USD、传感器安装位置和外参；
- sim/real 输入适配器；
- 官方 launch 的组合、参数覆盖和 topic remap；
- Nav2 到 locomotion policy 的安全接口；
- 可复现的启动配置和运行验收。

核心模型固定为：

```text
assets/my_B2Arx/my_b2arx/my_robot.usd
```

当前主链路：

```text
Isaac Sim 内嵌官方 ZED X ── ZED SDK stream ── 官方 zed_wrapper
                                               │ RGB/depth/pose/odom
                                               ▼
                                      Isaac ROS Nvblox
                                               │ map slice
                                               ▼
                                             Nav2
                                               │ /cmd_vel
                                               ▼
                                     watchdog + locomotion policy

Isaac Sim 内嵌 XT32 ── /lidar_points ── RViz / 其它点云算法
真实 Hesai XT32 ────── /lidar_points ── 同一算法接口

Isaac Sim 官方 D455 proxy ─┐
                            ├─ /wrist_camera/* ── AprilTag / Robot Segmenter / cuMotion
真实 Intel D435i ──────────┘
```

详细分层和 TF 所有权见 [系统架构](docs/architecture.md)。

Nav2 的障碍物输入有两个互斥模式。两种模式复用同一套 planner、controller、footprint、
`/b2/odom` 和 `cmd_vel` 安全链路，只替换 costmap 的障碍物来源：

| 模式 | 障碍物链路 | 是否启动 Nvblox | 默认定位来源 |
| --- | --- | --- | --- |
| `depth` | ZED registered depth → Isaac ROS Nvblox → `NvbloxCostmapLayer` | 是 | ZED VIO |
| `lidar` | XT32 `/lidar_points` → Nav2 `ObstacleLayer` | 否 | ZED VIO（可替换） |

`lidar` 模式中的 ZED 只负责当前系统的 VIO 和 `map -> base_link` 定位，不把深度或 RGB
送进 Nav2 costmap。若底盘/SLAM 已经提供 `/b2/odom` 和完整 TF，可以关闭 ZED 定位适配器。

## 上游复用

| 子系统 | 直接复用的官方入口 |
| --- | --- |
| ZED X | `zed_wrapper/launch/zed_camera.launch.py` |
| Nvblox | `nvblox_examples_bringup/launch/perception/nvblox.launch.py` |
| Nav2 | `nav2_bringup/launch/navigation_launch.py` |
| Hesai XT32 | `hesai_ros_driver_node` + 官方 YAML |
| RealSense D435i | `realsense2_camera/launch/rs_launch.py` |
| ARX / manipulation | `isaac_ros_manipulation_arx_r5a_bringup/launch/arx_r5a_apriltag_pick_and_place.launch.py` |

本仓库只做薄封装。项目专用代码目前只有 TF/odom、footprint、`cmd_vel` watchdog、Isaac Sim bridge 和策略适配。

## 本机工作区

当前机器已验证的默认位置：

```text
项目                  ~/b2arx_isaac_sim
Isaac ROS / ZED       ~/workspaces/isaac_ros-dev
Isaac ROS 源码        ~/workspace/isaac_ros_source
ARX manipulation      ~/workspace/isaac_ros_source/isaac_ros_manipulation_arx_r5a
Hesai 官方驱动        ~/hesai_ws
```

路径都可以用环境变量覆盖：

```bash
export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
export HESAI_WS="$HOME/hesai_ws"
export ROS_DOMAIN_ID=23
```

## 最短启动：仿真 + Nvblox + Nav2

### 终端 1：Isaac Sim、资产、传感器和策略执行器

在干净终端中启动，不要先 source ROS：

```bash
cd ~/b2arx_isaac_sim
conda activate isaaclab

./scripts/run_isaac_sim.sh \
  --config config/simulation/warehouse_nav2.yaml
```

该 profile 自动加载：

- 核心 `my_robot.usd`；
- warehouse 环境；
- 官方 ZED X SDK simulation stream；
- 官方 D455 Color prim 对应的腕部 RGB-D 接口；
- XT32 `/lidar_points`；
- `/clock` 和 `/cmd_vel`；
- `config/policies/basic_locomotion.yaml`。

等待终端出现：

```text
[READY]: Official Stereolabs ZED Streamer initialized successfully with ID ...
```

`stream graph configured` 只表示图已创建，不能作为 ROS receiver 的启动信号。

修改 YAML 后无需改 Python。临时 CLI 参数放在命令末尾即可覆盖 YAML：

```bash
./scripts/run_isaac_sim.sh \
  --config config/simulation/warehouse_nav2.yaml \
  --zed_stream_port 31000 \
  --headless
```

### 终端 2：官方 ZED、Isaac ROS Nvblox 和 Nav2

```bash
cd ~/b2arx_isaac_sim

./scripts/run_isaac_ros.sh sim --build \
  sim_address:=127.0.0.1 \
  sim_port:=30000 \
  use_rviz:=true
```

以后源码未变化时可省略 `--build`：

```bash
./scripts/run_isaac_ros.sh sim
```

Nav2 的障碍物来源已经拆成两个独立模式，planner、controller、footprint、
watchdog 和 `/b2/odom` 接口保持完全一致：

```bash
# ZED registered depth -> Isaac ROS Nvblox -> NvbloxCostmapLayer（默认）
./scripts/run_isaac_ros.sh sim navigation_mode:=depth

# XT32 /lidar_points -> Nav2 ObstacleLayer，不启动 Nvblox
./scripts/run_isaac_ros.sh sim navigation_mode:=lidar
```

也可以直接调用对应的算法入口：

```bash
# 深度模式
ros2 launch b2arx_nav2_bringup isaac_ros_nav2.launch.py sensor_mode:=sim

# 雷达模式
ros2 launch b2arx_nav2_bringup b2arx_xt32_nav2.launch.py sensor_mode:=sim
```

雷达模式当前仍默认启动 ZED，但只用其 VIO 提供 `/b2/odom` 和
`map -> base_link` 定位链；ZED 深度、RGB 和 Nvblox 都不进入 costmap。
如果以后由机器人本体或其它定位系统提供 `/b2/odom` 及完整 TF，可同时设置
`start_zed_wrapper:=false start_odometry_adapter:=false`。

启动后可用下面的命令确认实际使用的障碍层：

```bash
# 深度模式应包含 nvblox_layer；雷达模式应包含 obstacle_layer
ros2 param get /local_costmap/local_costmap plugins

# 雷达模式应看到 PointCloud2 publisher 和 Nav2 costmap subscriber
ros2 topic info /lidar_points --verbose

# 自动检查对应模式的 topic、TF、生命周期和 costmap 参数
./scripts/check_isaac_ros_runtime.sh --navigation-mode depth
./scripts/check_isaac_ros_runtime.sh --navigation-mode lidar
```

XT32 的高度和距离过滤位于
`ros_ws/src/b2arx_nav2_bringup/config/b2arx_nav2_xt32.yaml`；真机首次运行应根据地面
点云的实际高度复核 `min_obstacle_height` / `max_obstacle_height`。该模式是实时
marking/clearing costmap，不会自动生成持久化 SLAM 地图。

旧命令仍保留兼容：

```bash
ros2 launch b2arx_nav2_bringup b2arx_zed_nvblox_nav2.launch.py
```

### 终端 3：操作算法子系统（按需）

腕部相机、Nav2 和操作算法是独立模块。真实 D435i 的图像已经在终端 2 启动时，另开终端 source 现有 manipulation workspace 和本项目 overlay：

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/isaac_ros_source/isaac_ros_manipulation_arx_r5a/install/setup.bash
source ~/b2arx_isaac_sim/ros_ws/install/local_setup.bash

ros2 launch b2arx_nav2_bringup \
  manipulation_wrist_d435i.launch.py
```

这个 launch 只做 D435i topic、frame、外参和行为树 frame 的适配；AprilTag、Robot Segmenter、Nvblox、cuMotion、机械臂驱动和 pick-and-place 编排仍来自现有 `isaac_ros_manipulation_arx_r5a`。首次运行应保持安全区、先关闭自动执行或使用上游 plan-only 流程，确认 TF 和规划结果后再下发真机动作。

### 关于“策略单独一个终端”

当前策略以 200 Hz 直接读取并写入 Isaac articulation，因此执行器仍在终端 1 的 Isaac Sim 进程内。模型和部署配置已经独立；真正拆成 ROS 进程还需要 joint state、IMU、根状态、关节命令、时间同步和 watchdog 的完整接口。

## 真机启动

算法层与仿真相同，只替换 ZED/Hesai/RealSense/机器人输入。

真实 XT32 配置当前位于：

```text
~/hesai_ws/src/HesaiLidar_ROS_2.0/config/config.yaml
```

其中已配置：

```text
topic: /lidar_points
frame: hesai_lidar
correction_file_path: XT32 angle correction CSV
firetimes_path: official PandarXT-32 firetime CSV
```

firetime 和 angle correction CSV 只由真实 Hesai 驱动使用，Isaac Sim 不需要。

启动真机 ZED + Hesai + Nvblox + Nav2：

```bash
cd ~/b2arx_isaac_sim

export HESAI_CONFIG_FILE="$HOME/hesai_ws/src/HesaiLidar_ROS_2.0/config/config.yaml"

./scripts/run_isaac_ros.sh real --build \
  start_hesai:=true \
  start_wrist_realsense:=true \
  serial_number:=0 \
  use_rviz:=true
```

上面的默认仍是 ZED depth + Nvblox。真机切到 XT32 直接避障只需增加：

```bash
./scripts/run_isaac_ros.sh real \
  navigation_mode:=lidar \
  start_hesai:=true \
  use_rviz:=true
```

`wrist_realsense_serial` 默认不硬编码。只有多台 RealSense 时才选择设备，纯数字序列号按官方驱动格式加前导下划线：

```bash
./scripts/run_isaac_ros.sh real \
  start_wrist_realsense:=true \
  wrist_realsense_serial:=_YOUR_NUMERIC_SERIAL
```

也可以完全独立启动腕部相机，不带 ZED/Nvblox/Nav2：

```bash
source /opt/ros/jazzy/setup.bash
source ~/b2arx_isaac_sim/ros_ws/install/local_setup.bash
ros2 launch b2arx_nav2_bringup wrist_realsense.launch.py \
  start_wrist_realsense:=true
```

如果真实底盘已经直接发布标准 `/b2/odom`，关闭 ZED odometry adapter：

```bash
./scripts/run_isaac_ros.sh real \
  start_odometry_adapter:=false \
  start_hesai:=true
```

如果机器人 URDF 已发布 ZED 或 XT32 安装 TF，还要关闭对应静态 TF，避免双发布：

```bash
publish_zed_mount_tf:=false publish_hesai_tf:=false
```

## ROS launch 模块

```text
ros_ws/src/b2arx_nav2_bringup/launch/
├── zed.launch.py                    官方 ZED sim/real adapter
├── hesai_xt32.launch.py             官方 Hesai executable adapter
├── wrist_realsense.launch.py        官方 RealSense D435i include
├── nvblox.launch.py                 官方 Nvblox include
├── nav2.launch.py                   官方 Nav2 include + B2 速度安全节点
├── platform_adapters.launch.py      配置化 B2ARX TF、odom adapter 和 RViz
├── manipulation_wrist_d435i.launch.py  现有 ARX manipulation 的腕部 profile
├── isaac_ros_nav2.launch.py         共用算法总入口
├── b2arx_xt32_nav2.launch.py        XT32 ObstacleLayer 独立导航入口
├── bringup_sim.launch.py            仿真 profile
├── bringup_real.launch.py           真机 profile
└── b2arx_zed_nvblox_nav2.launch.py  旧入口兼容层
```

常用模块可独立启动：

```bash
ros2 launch b2arx_nav2_bringup zed.launch.py sensor_mode:=sim
ros2 launch b2arx_nav2_bringup nvblox.launch.py
ros2 launch b2arx_nav2_bringup nav2.launch.py
ros2 launch b2arx_nav2_bringup platform_adapters.launch.py
ros2 launch b2arx_nav2_bringup wrist_realsense.launch.py
ros2 launch b2arx_nav2_bringup manipulation_wrist_d435i.launch.py
```

`nav2.launch.py` 独立调用时默认 `use_composition:=false`，由官方 Nav2
launch 直接启动各个进程，不依赖 `/nvblox_container`。只有在外部容器已经
存在时才显式使用 `use_composition:=true container_name:=...`。

总入口 `isaac_ros_nav2.launch.py` 固定让 Nav2 composition 复用官方 Nvblox
创建的唯一容器，因此 `start_nav2:=true` 必须同时保持
`start_nvblox:=true`；不满足时会在启动阶段直接报错，而不会等待不存在的容器。

## 配置入口

| 配置 | 作用 |
| --- | --- |
| `config/simulation/warehouse_nav2.yaml` | 完整仿真、ZED、腕部 RGB-D、XT32、ROS domain、policy |
| `config/simulation/warehouse.yaml` | 交互场景和全部传感器，不自动进入 Nav2 policy 模式 |
| `config/simulation/minimal.yaml` | 无 ROS/传感器流的快速视觉 smoke test |
| `config/policies/basic_locomotion.yaml` | locomotion 模型、部署参数和 `/cmd_vel` 输入 |
| `ros_ws/src/b2arx_nav2_bringup/config/zedx_nvblox_release_4_5.yaml` | ZED 项目覆盖 |
| `ros_ws/src/b2arx_nav2_bringup/config/nvblox_b2arx.yaml` | Nvblox 项目覆盖 |
| `ros_ws/src/b2arx_nav2_bringup/config/b2arx_nav2.yaml` | Nav2、costmap、watchdog 和 footprint |
| `ros_ws/src/b2arx_nav2_bringup/config/b2arx_nav2_xt32.yaml` | XT32 `/lidar_points` Nav2 ObstacleLayer 模式；非 costmap 参数与深度模式保持一致 |
| `ros_ws/src/b2arx_nav2_bringup/config/platform_adapters.yaml` | ZED/XT32 安装外参、frame 和 odometry adapter |
| `ros_ws/src/b2arx_nav2_bringup/config/wrist_realsense_d435i.yaml` | 官方 RealSense 真机流参数 |
| `config/arx_r5a_d543if_eih.calib` | easy_handeye 原始测量结果，保留原格式 |
| `ros_ws/src/b2arx_nav2_bringup/config/wrist_d435i_eye_in_hand.yaml` | 供现有 manipulation launch 使用的生成外参 |

优先级：显式 CLI/launch argument > 项目 YAML > 上游默认配置。

## 稳定 ROS 接口

### ZED X

```text
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/camera_info
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/camera_info
/zed/zed_node/pose
/zed/zed_node/odom
```

当前 RTX 5090/CUDA 13 主机默认 `disable_zed_nitros:=true`，因为 ZED SDK 5.4 Managed NITROS 路径曾稳定触发 Xid 31/CUDA 700。Nvblox 仍是官方 Isaac ROS 节点，只是图像暂时走稳定的 `sensor_msgs/Image`。升级驱动/SDK 后可显式 A/B：

```bash
./scripts/run_isaac_ros.sh sim disable_zed_nitros:=false
```

### B2 odometry

```text
input:  /zed/zed_node/odom，child=zed_camera_link
output: /b2/odom，child=base_link
```

`odometry_adapter` 会复合相机安装外参，并对旋转运动进行杆臂速度修正。它与静态
`zed_camera_link -> base_link` TF 共用 `config/platform_adapters.yaml` 中同一组
translation/quaternion，避免两处外参漂移。Nav2 和 velocity smoother 只订阅 `/b2/odom`。

### XT32

```text
topic: /lidar_points
type: sensor_msgs/msg/PointCloud2
frame: hesai_lidar
```

Nvblox 当前只融合 ZED depth。Isaac ROS 4.5 官方 ZED specialization 明确拒绝同实例 ZED + LiDAR 融合，因此 XT32 保持独立，供 RViz 和其它点云算法使用。

需要让 XT32 直接承担 Nav2 避障时，选择 `navigation_mode:=lidar`。该模式使用
官方 `nav2_costmap_2d::ObstacleLayer` 直接消费 `/lidar_points`，不会把 XT32
强行塞进 ZED Nvblox 实例。它提供实时 marking/clearing costmap，并不等价于
持久化 SLAM 地图；需要跨视野长期保存的全局地图时，应另接 SLAM/地图服务器。

### 速度

```text
Nav2 velocity smoother -> cmd_vel_watchdog -> /cmd_vel -> policy
```

`/cmd_vel` 必须只有 watchdog 一个发布者；超时后持续发布零速度并同步 heartbeat。

## D435i / 腕部相机

真实机器人使用 Intel D435i。Isaac Sim 5.1 官方资产中没有 D435i USD，核心模型里保留的是官方 D455，所以仿真端明确把它当作“D435i ROS 接口代理”，不宣称两种硬件的基线、内参、噪声或 IMU 模型完全一致。

sim/real 使用同一组算法话题：

```text
/wrist_camera/color/image_raw
/wrist_camera/color/camera_info
/wrist_camera/aligned_depth_to_color/image_raw
/wrist_camera/aligned_depth_to_color/camera_info
```

仿真端使用 Isaac Sim 官方 `ROS2CameraHelper` 和 `ROS2CameraInfoHelper`。RGB 与 depth 绑定 D455 的同一个 Color camera render product，因此是真正同光心的 aligned depth；不会把相对 Color 光心有偏移的 `Camera_Pseudo_Depth` 冒充 aligned depth。真机端直接 include 官方 `realsense2_camera/rs_launch.py`，启用 color、depth、sync 和 `align_depth.enable`。

深度值的单位不能只凭 topic 名判断：

| 模式 | 常见 encoding | 数值单位 / scale |
| --- | --- | --- |
| Isaac Sim | `32FC1` | 米，scale `1.0` |
| D435i 真机 | `16UC1` | 毫米，换算米通常乘 `0.001` |

下游算法应按运行 profile 或消息 encoding 选择 scale，不能把仿真的浮点米值再乘 `0.001`。

### easy_handeye 外参

原始标定文件是 `config/arx_r5a_d543if_eih.calib`。easy_handeye 的 eye-in-hand 结果方向已核实为：

```text
link6 -> D543if_link
```

适配新驱动命名后为 `link6 -> wrist_camera_link`，数值原样保留，不能求逆。官方 RealSense 驱动继续拥有 `wrist_camera_link -> wrist_camera_color_optical_frame` 的内部 TF。重新标定后生成并校验 manipulation 配置：

```bash
cd ~/b2arx_isaac_sim
python3 scripts/easy_handeye_to_manipulation.py
python3 scripts/easy_handeye_to_manipulation.py --check
```

这个真实标定只应发布在与该实物安装一致的真机链路；仿真 USD 的安装位姿仍以 authored USD 为准，除非你确认两者被刻意做成完全相同。

### 话题和 RViz 检查

```bash
ros2 topic hz /wrist_camera/color/image_raw
ros2 topic hz /wrist_camera/aligned_depth_to_color/image_raw
ros2 topic echo /wrist_camera/color/camera_info --field header.frame_id --once
ros2 topic echo /wrist_camera/aligned_depth_to_color/image_raw \
  sensor_msgs/msg/Image --field encoding --once
ros2 run tf2_ros tf2_echo link6 wrist_camera_color_optical_frame
```

在 RViz2 中添加两个 `Image` display 并分别选择 color/depth topic；点云仍添加 `PointCloud2` 并选择 `/lidar_points`。

## 构建和检查

手动构建 ROS overlay：

```bash
source /opt/ros/jazzy/setup.bash
source "${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}/install/setup.bash"

cd ~/b2arx_isaac_sim/ros_ws
colcon build --symlink-install --packages-select b2arx_nav2_bringup
source install/local_setup.bash
```

静态回归：

```bash
cd ~/b2arx_isaac_sim
pytest -q
```

依赖预检必须在 `isaac-ros activate` 创建的官方容器终端中运行：

```bash
isaac-ros activate
# 出现 (isaac-ros) 提示符后：
cd ~/b2arx_isaac_sim
./scripts/check_isaac_ros_4_5.sh
```

`./scripts/run_isaac_ros.sh sim --preflight` 同样要求从已经 activate 的终端执行。

在线运行验收：

```bash
# 深度/Nvblox 模式（默认）
./scripts/check_isaac_ros_runtime.sh --navigation-mode depth --require-wrist-camera

# XT32/ObstacleLayer 模式
./scripts/check_isaac_ros_runtime.sh --navigation-mode lidar
```

两种模式都会检查定位、`/b2/odom`、XT32、TF、Nav2 lifecycle/costmap 和
`/cmd_vel` 唯一发布者；只有 `depth` 模式检查 ZED depth/color 到 Nvblox 的连接及
map slice，`lidar` 模式则检查 `/lidar_points` 的 Nav2 subscriber。不需要检查腕部
相机时可省略 `--require-wrist-camera`。

## 项目目录

```text
assets/          核心机器人、环境和道具资产
config/          仿真、策略等非 ROS 配置
models/          可部署策略 bundle
scripts/         稳定启动入口、bridge 和诊断工具
ros_ws/src/      B2ARX ROS bringup/adapter 包
tests/           单元、合同和集成测试
docs/            架构、标定和工作流说明
outputs/         运行输出，默认忽略
```

生成的 `build/install/log`、缓存、数据库和抓帧输出不属于源码。核心资产、模型和用户导入的环境/道具不会被自动清理。

## 常见问题

### ZED receiver 连不上

- 先等 Isaac Sim 打印 streamer `READY`；
- 两端端口必须一致且为可用偶数端口；
- 同机用 `127.0.0.1`；
- 两机部署时，`sim_address` 填 Isaac Sim 主机可达地址，不是 receiver 自己的地址。

### ROS 能看到 topic 名但没有图像

topic 名可能由 subscriber 端点创建。使用：

```bash
ros2 topic info /zed/zed_node/depth/depth_registered --verbose
```

必须同时存在 `zed_node` publisher 和 `nvblox_node` subscriber。

### RViz 看不到 XT32

```bash
ros2 topic echo /lidar_points --field header.frame_id --once
ros2 run tf2_ros tf2_echo base_link hesai_lidar
```

预期 frame 为 `hesai_lidar`，并存在 `base_link -> hesai_lidar`。

### Ctrl-C 后共享容器退出为 -11

Isaac ROS 4.5 的官方 Nvblox 与 composed Nav2 共用容器时，收到 Ctrl-C 后可能在组件析构阶段
以 `-11` 退出。这是当前上游关停路径的已知行为，不等同于运行期间 Nvblox/Nav2 已失败。
判断时应先确认 `-11` 是否只发生在用户发送 SIGINT 之后，并结合运行验收脚本、lifecycle 状态、
map slice 和 costmap 是否在关停前持续正常。若进程在未 Ctrl-C 时自行 `-11`，仍应按真实运行故障排查。

### 修改环境或核心 USD

优先改 `config/simulation/*.yaml`。核心机器人始终指向：

```text
assets/my_B2Arx/my_b2arx/my_robot.usd
```

在 Isaac Sim GUI 中修改后保存到该 USD 或另存新文件，再修改 profile 的 `robot_usd`；不要在运行脚本中重新拼装传感器资产。

## 相关文档

- [系统架构与 sim/real 合同](docs/architecture.md)
- [ZED 安装和挂载工作流](docs/zed_mount_workflow.md)
- [末端目标转换](docs/ee_target_pipeline.md)
