# 核心 USD 中的 ZED X 与 XT32

## 唯一核心资产

以后仿真统一加载下面这个机器人资产：

```text
assets/my_B2Arx/my_b2arx/my_robot.usd
```

ZED X 和 Hesai XT32 都已经保存到这个 USD 内，场景代码只绑定现有 prim，不再生成第二套传感器，
也不再使用 Robot Assembler 或运行时 Fixed Joint。加载到 IsaacLab 场景后的关键路径为：

```text
/World/envs/env_0/Robot/b2_description/R5a/ZED_X
  /base_link/ZED_X/CameraLeft
  /base_link/ZED_X/CameraRight
  /base_link/ZED_X/Imu_Sensor

/World/envs/env_0/Robot/b2_description/XT_32/PandarXT_32_10hz
```

ZED 根 prim 是机器人刚体上的非刚体 compound；官方传感器内部碰撞仍保留。这种结构既避免嵌套刚体
警告，也让外观、碰撞、相机和 IMU 随机器人一起运动。

## 当前坐标合同

机器人使用 `+X` 前、`+Y` 左、`+Z` 上。核心 USD 中：

```text
R5a -> ZED_X root          = (0.280, 0.000, -0.020), rotation identity
R5a -> left optical center = (0.295,+0.060, -0.005)
R5a -> right optical center= (0.295,-0.060, -0.005)
ZED stereo baseline        = 0.120 m

base_link -> XT32 sensor   = (0.3493326833, 0.000, 0.1603575616), rotation identity
```

ROS 中 ZED wrapper 自己维护 `odom -> zed_camera_link`。为把它接回 B2，bringup 发布：

```text
zed_camera_link -> base_link = (-0.525, 0.000, -0.079), rotation identity
base_link -> hesai_lidar     = (+0.3493326833, 0.000, +0.1603575616), rotation identity
```

## 在 Isaac Sim GUI 中继续手调

应直接打开 `my_robot.usd` 修改源资产，而不是在运行时克隆出来的 `/World/envs/env_0/Robot` 上保存
session override：

1. 暂停 Timeline，通过 `File > Open` 打开 `assets/my_B2Arx/my_b2arx/my_robot.usd`。
2. 调 ZED 安装位姿时，选择 `b2_description/R5a/ZED_X`，把工具栏坐标模式切到 `Local`，修改该根
   Xform；不要单独移动 `CameraLeft`、`CameraRight` 或 `Imu_Sensor`。
3. 调 XT32 时选择 `b2_description/XT_32`，不要修改内部 `PandarXT_32_10hz` 的扫描配置或 prim 名。
4. 用 `File > Save` 写回同一个核心 USD。保留上述 prim 路径，否则启动时的传感器合同检查会明确报错。

保存后可检查组合位姿和 ZED 左目视角：

```bash
conda activate isaaclab
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --viewer_camera zed_left --print_zed_debug
```

`--save_zed_frames` 保存的只是 renderer 侧 RGB 几何检查图。正式 RGB、深度、CameraInfo、VIO 和 TF
仍由官方 ZED SDK simulation stream 与 `zed_wrapper` 产生；代码已将 streamer 直接绑定到内嵌
`.../R5a/ZED_X`。

streamer 默认使用官方 `BOTH` transport（网络 stream + Linux 本机 IPC）。也可以在场景 CLI 中显式选择
`--zed_stream_transport NETWORK`（仅网络，支持本机或远端 receiver）或
`--zed_stream_transport IPC`（仅 Linux 本机 receiver）；三个可选值是 `BOTH`、`NETWORK`、`IPC`，默认
为 `BOTH`。Isaac Sim 与 ROS receiver 仍需使用相同的偶数 stream 端口。

## XT32 PointCloud2 与 RViz

给场景加 `--ros2` 后，Isaac Sim 会对内嵌官方 `OmniLidar` 建立 RTX LiDAR graph，并默认发布：

```text
topic: /lidar_points
type:  sensor_msgs/msg/PointCloud2
frame: hesai_lidar
scan:  32 x 2000, nominal 10 Hz in simulation time
fields: x, y, z (FLOAT32)
```

这与真机 Hesai driver 的 topic/frame 合同一致。仿真 writer 没有 `intensity`、`ring` 或逐点时间戳，
因此 RViz 使用 `AxisColor`。Isaac ROS 4.5 的 ZED specialization 不支持同时融合 LiDAR；当前点云独立
发布，不进入这个 ZED Nvblox 实例。

独立检查点云：

```bash
export ROS_DOMAIN_ID=23
ros2 topic info -v /lidar_points
ros2 topic echo /lidar_points --once
ros2 topic hz /lidar_points
```

完整扫描名义周期是 `0.1 s` 仿真时间；`ros2 topic hz` 显示的是墙钟频率，会随 Isaac Sim 的实时因子
变化。首包需要等待一整圈扫描和 writer 初始化。

完整 bringup 会发布 `base_link -> hesai_lidar`，并在仓库 RViz 配置中默认显示独立的
`XT32 Point Cloud`；当前 Nvblox 以 ZED registered depth 为唯一重建深度输入
（`use_depth=true`、`use_lidar=false`），XT32 不进入这个 Nvblox 实例：

```bash
ros2 launch b2arx_nav2_bringup b2arx_zed_nvblox_nav2.launch.py use_rviz:=true
```

若只启动 Isaac Sim 而不启动 bringup，可单独运行 RViz，并暂时把 `Fixed Frame` 设为
`hesai_lidar` 来直接看原始点云。
