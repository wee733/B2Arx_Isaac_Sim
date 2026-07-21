# B2ARX Isaac Sim

这是 B2 机器狗 + ARX R5 机械臂的 Isaac Lab 仿真工作区。仓库把机器人、官方相机资产、训练策略和
Isaac ROS 导航链路放在同一个可复现的仿真入口中，目标是先在仿真验证真实机器人上已经跑通的感知、建图、
导航和策略部署流程，再接入后续的语义分割/操作算法。

> **当前状态（2026-07）**：官方 Stereolabs ZED X 仿真流 → `zed_wrapper` → Isaac ROS Nvblox →
> Nav2 → `Twist [vx, vy, wz]` → 可替换的 locomotion policy → B2 关节控制，已经完成端到端验证。
> 这里的仿真相机必须写作 **ZED X**：Stereolabs Isaac Sim Extension `v4.3.0` 没有 ZED 2/2i 的官方
> USD/streamer model。ZED 2i 的外观或 TF 可以作为模型参考，但不能冒充官方 ZED SDK 深度仿真。

文中的绝对路径均写成可替换变量，避免把开发机用户名或目录写死。首次使用时，在每个需要运行命令的
终端设置：

```bash
export B2ARX_SIM_ROOT="$(git rev-parse --show-toplevel)"
export ISAACLAB_ROOT="/path/to/IsaacLab"
export B2ARX_TRAIN_ROOT="/path/to/b2arx"             # checkpoint/export 工作区
export ARX_ID_ROOT="/path/to/arx_actuator_identification"  # 可选：辨识 PD 参数
export ISAAC_ROS_ROOT="/path/to/b2arx_isaac_ros"     # ROS/SDK overlay 根目录
export B2ARX_WAREHOUSE_USD="/path/to/Simple_Warehouse/warehouse.usd"  # 可选：离线场景
export ZED_PORT="${ZED_PORT:-30000}"                # 偶数端口；两端必须相同
```

`/path/to/...` 只是占位符，请替换为本机实际路径；后文命令不依赖特定用户名。

## 先看这里：能力和运行合同

| 模块 | 当前实现 | 已验证接口 |
| --- | --- | --- |
| 动力学 | Isaac Sim 5.1 + Isaac Lab | B2 + ARX R5、200 Hz 物理步 |
| 前置立体相机 | Stereolabs 官方 ZED X USD/Extension `v4.3.0` | SDK simulation stream，`HD1200 @ 30 Hz` |
| 腕部相机 | Isaac Sim 官方 RealSense D455 USD | RGB、深度、双红外 |
| 感知/建图 | `zed_wrapper 5.4` + Isaac ROS Nvblox release 4.5 | 深度、CameraInfo、VIO、mesh/map slice |
| 导航 | Nav2 Jazzy | `SmacPlanner2D` + RPP + XY-only goal checker |
| 运动控制 | YAML 指定的 locomotion policy（参考 baseline 为 `model_29999`） | `/cmd_vel` 的 `[vx, vy, wz]` 输入 |

完整数据流如下；Isaac Sim 不自行伪造 ZED ROS 话题，ZED 图像、深度、CameraInfo、位姿和 TF 均由
官方 `zed_wrapper` 从 SDK 仿真流产生：

```text
ZED_X.usdc + ZED Camera Helper
        │  Stereolabs SDK simulation stream (NETWORK/IPC, even port)
        ▼
zed_wrapper (sim_mode=true)
        ├── RGB / depth / CameraInfo / pose / TF
        ▼
Isaac ROS Nvblox ──► Nav2 costmaps ──► SmacPlanner2D + RPP
                                      │  geometry_msgs/Twist
                                      ▼
                          cmd_vel watchdog + ros2_twist
                                      ▼
                         locomotion policy → B2 joints
```

### 推荐阅读顺序

1. [双终端启动（ZED + Nvblox + Nav2）](#zednvbloxnav2-双端启动)
2. [检查建图和 Nav2](#检查建图和-nav2)
3. [发送已验证的 NavigateToPose](#发送已验证的-navigatetopose)
4. [策略部署合同](#策略部署)
5. [当前仿真限制](#当前仿真限制)

如果只想确认机器人和相机能显示，先看[启动场景](#启动场景)；如果要复现实验结果，直接按上面第 1 项
启动两个终端。

## 最短可运行路径（Nav2）

以下步骤是仓库当前的最小端到端路径。需要预先准备：Isaac Sim 5.1/Isaac Lab `isaaclab` 环境、
ROS 2 Jazzy、Isaac ROS Nvblox release 4.5、ZED SDK 5.4、官方 ZED Isaac Sim Extension，以及
Git LFS。仓库已包含 `basic_locomotion` 的 `.pt`、ONNX、部署参数和 manifest；首次 clone 后先拉取
LFS 模型文件，再安装扩展：

```bash
cd "$B2ARX_SIM_ROOT"
git lfs install
git lfs pull
python3 scripts/install_zed_isaac_sim.py
```

首次 clone 或修改 `ros_ws/src/b2arx_nav2_bringup` 后，构建一次工作区 overlay：

```bash
cd "$B2ARX_SIM_ROOT/ros_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select b2arx_nav2_bringup
source install/local_setup.bash
```

然后开两个终端。两端的 `ROS_DOMAIN_ID`、`sim_address`、`sim_port` 必须一致；`NETWORK` 模式下
`sim_address` 填运行 Isaac Sim 的实际物理网卡地址，不要默认写 `127.0.0.1`。

终端 A（Isaac Sim、ZED streamer 和策略）：

```bash
cd "$B2ARX_SIM_ROOT"
conda activate isaaclab

"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --nav2 --ros2_domain_id 23 \
  --scene_asset warehouse --no_workspace --no_scene_camera \
  --viewer_camera scene --print_policy_debug \
  --zed_stream_transport NETWORK --zed_stream_port "$ZED_PORT"
```

等待 Isaac 窗口中机器人稳定站立，并看到 `Official Stereolabs ZED SDK stream graph active` 后再启动
终端 B。默认端口是 `30000`；如果它已被占用，给 `ZED_PORT` 换成任意未占用的偶数端口
（详见[端口故障排查](#端口和重启故障排查)）。

终端 B（官方 `zed_wrapper`、Nvblox 和 Nav2）：

```bash
cd "$B2ARX_SIM_ROOT"
source /opt/ros/jazzy/setup.bash

# 下面四个路径按本机 Isaac ROS 安装位置调整；这是已验证的 overlay 结构。
APT_ROOT="$ISAAC_ROS_ROOT/apt_root"
ZED_SDK_ROOT="$ISAAC_ROS_ROOT/zed_sdk_5_4"
export AMENT_PREFIX_PATH="$APT_ROOT/opt/ros/jazzy${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$APT_ROOT/opt/ros/jazzy${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PATH="$APT_ROOT/opt/ros/jazzy/bin:$PATH"
export LD_LIBRARY_PATH="$APT_ROOT/opt/ros/jazzy/lib:$APT_ROOT/opt/ros/jazzy/lib/x86_64-linux-gnu:$APT_ROOT/usr/lib/x86_64-linux-gnu:$ZED_SDK_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$APT_ROOT/opt/ros/jazzy/lib/python3.12/site-packages:$APT_ROOT/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export ZED_SDK_SETTINGS_PATH="$ZED_SDK_ROOT/settings/"
export ROS_DOMAIN_ID=23 LC_ALL=C
source "$B2ARX_SIM_ROOT/ros_ws/install/local_setup.bash"

ISAAC_HOST_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')"
test -n "$ISAAC_HOST_IP" || { echo "请手动设置 ISAAC_HOST_IP 为 Isaac Sim 的物理网卡 IPv4"; exit 1; }
ros2 launch b2arx_nav2_bringup b2arx_zed_nvblox_nav2.launch.py \
  domain_id:=23 sim_address:="$ISAAC_HOST_IP" sim_port:="$ZED_PORT" \
  use_rviz:=true log_level:=info
```

在 RViz 里确认 `Nvblox Mesh`/costmap 有数据后，用 `2D Goal Pose` 发布目标。也可以不启动 RViz，
在终端 B 中用 `/navigate_to_pose` action 发送目标，示例和成功判据见[发送已验证的 NavigateToPose](#发送已验证的-navigatetopose)。

### 最小验收标准

下列条件同时满足，才算完整链路已经启动，而不只是 Isaac 窗口显示了相机：

- `zed_wrapper` 能持续发布 `/zed/zed_node/depth/depth_registered`、CameraInfo 和 `/zed/zed_node/pose`；
- `/nvblox_node/static_map_slice` 持续有消息，Nav2 lifecycle 节点为 `active [3]`；
- 空闲时 `/cmd_vel` 持续发布全零，`/cmd_vel_heartbeat` 序号递增；
- 发送 XY 目标后 `/cmd_vel` 出现带平移的弧线速度（典型 `vx=0.25 m/s`、`wz` 在训练范围内），
  不出现纯 `vx=0, vy=0, wz!=0`；
- NavigateToPose 返回 `error_code=0`，机器人到达 `xy_goal_tolerance=0.30 m` 内后速度归零。

## 项目结构与本地依赖

核心源码和配置位于以下位置；`outputs/`、`logs/` 以及 `ros_ws/build|install|log` 是运行时生成目录，
已由 `.gitignore` 排除，不应提交到仓库：

```text
assets/my_B2Arx/                 B2 + ARX R5 的 URDF、mesh 和生成 USD
assets/isaac_sensors/            随仓库保留的官方 D455 依赖材质/资产
models/basic_locomotion/         基线 .pt、ONNX、deploy.yaml 与 hash manifest（Git LFS）
scripts/isaac_b2arx_scene.py     Isaac Sim 场景、相机挂载和 ZED streamer
scripts/policy_deploy/            ONNX 策略、FSM、ROS Twist 适配器
config/policies/                 可切换模型的策略 profile（默认 basic_locomotion）
ros_ws/src/b2arx_nav2_bringup/   ZED wrapper、Nvblox、Nav2 一体化 launch/config/RViz
tests/                            策略、命令适配和 watchdog 的离线测试
```

- Isaac Lab：`${ISAACLAB_ROOT}`
- Isaac 训练工作区：`${B2ARX_TRAIN_ROOT}`
- 本项目工作区：`${B2ARX_SIM_ROOT}`
- 本地机器人资产：`assets/my_B2Arx`
- 默认机器人 USD：`assets/my_B2Arx/my_b2arx/my_robot.usd`
- 合并 URDF 源文件：`assets/my_B2Arx/my_robot.urdf`
- Conda 环境：`isaaclab`

Nav2 端还需要 ROS 2 Jazzy、Isaac ROS Nvblox release 4.5 的 `nvblox_ros`/`nvblox_nav2`、
Stereolabs ZED SDK 5.4 和 `zed_wrapper` 5.4。仓库的 launch 会显式设置 `use_sim_time=true`；不要把
真实 ZED 的第二个 driver 与本仓库的仿真 `zed_wrapper` 同时启动。

## 启动场景

```bash
cd "$B2ARX_SIM_ROOT"
conda activate isaaclab
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py
```

默认模式是 `hold`：每个物理步都会给机器人发送初始关节目标，让 B2+R5 不会因为重力直接塌下去。这个默认姿态和训练 / sim2sim 的初始设计对齐，机械臂 PD 默认使用最新的系统辨识纯 PD 参数。

- B2 站立姿态：FL/RL 为 `[+0.15, +0.67, -1.32]`，FR/RR 为 `[-0.15, +0.67, -1.32]`
- R5 机械臂姿态：`[0.0, 1.0, 0.8, 0.0, 0.0, 0.0]`
- 物理步长：`dt=0.005`，也就是 200 Hz
- 腿部 PD：hip/thigh `kp=300,kd=7.5`，calf `kp=500,kd=12.5`
- 机械臂默认 PD：`--arm_gain_profile identified`，来源是
  `${ARX_ID_ROOT}/arx_id_data/<fit_run>/fit_out/actuator_params_isaac.yaml`

如果想和训练时的高增益 arm profile 对比：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --arm_gain_profile train
```

## 策略部署

场景现在可以不再只固定 demo 姿态，而是加载导出的 B2+R5 策略。部署链路对齐训练仓的
`b2arx_sim2sim2real` 控制设计：

### 模型替换与配置入口

策略是可替换的，运行时不直接把 `.pt` 当作推理输入：`.pt` 是训练/导出的源文件，Isaac Sim 实际加载
`policy_full.onnx` 和其配套的 `params/deploy.yaml`。仓库提供完整的
`models/basic_locomotion/` 基线 bundle：训练来源是 `model_29999.pt`，在此仓库中重命名为
`basic_locomotion_model.pt`。模型二进制由 Git LFS 管理。

常调参数集中在配置文件和 launch 参数中，不需要改 Python 控制逻辑：

| 入口 | 用途 |
| --- | --- |
| `config/policies/basic_locomotion.yaml` | Nav2 默认 profile：模型 bundle、`[vx, vy, wz]` 限制、heartbeat 和 FSM |
| `scripts/policy_deploy/deploy_config.example.yaml` | 非 Nav2 的 keyboard/gamepad/scripted policy smoke test |
| `ros_ws/src/b2arx_nav2_bringup/config/b2arx_nav2.yaml` | planner/controller、RPP、costmap、goal checker、速度平滑器 |
| `ros_ws/src/b2arx_nav2_bringup/config/zedx_nvblox_release_4_5.yaml` | 官方 `zed_wrapper` 仿真参数和 ZED→Nvblox remapping |
| `b2arx_zed_nvblox_nav2.launch.py` | `domain_id`、`sim_address`、`sim_port`、`use_rviz`、`log_level` 等运行参数 |
| `scripts/isaac_b2arx_scene.py` CLI | 场景、ZED stream、viewer、workspace 和控制模式开关 |

替换模型时按下面的顺序操作：

1. 在训练仓把目标 checkpoint 导出成标准部署目录（`exported/policy_full.onnx`、`params/deploy.yaml`）。
2. 将 checkpoint、ONNX、`deploy.yaml` 和 manifest 放入 `models/<model-name>/`；复制
   `config/policies/basic_locomotion.yaml` 并把相对路径指向同一完整 bundle。
3. 根据新策略的 deploy 合同更新 YAML 中的 observation/history、action scale、EMA 和速度边界；Nav2
   RPP 只能发送该策略实际训练过的 `[vx, vy, wz]` 域。
4. 先运行离线测试，再启动 `--nav2`。启动器会校验 manifest、checkpoint、ONNX 和 deploy YAML 的
   SHA-256；校验失败时应修正 bundle，而不是关闭校验。

- 本独立场景不直接加载训练 checkpoint `.pt`
- `.pt` 要先在训练仓导出为 `exported/policy_full.onnx` 和 `params/deploy.yaml`
- 使用嵌套的 `params/deploy.yaml`
- 使用 `exported/policy_full.onnx`
- 单帧 observation 是 73 维
- history 是 30 帧
- action 是 18 维：12 个 B2 腿部关节 + `R5a_joint1~6`
- FSM 为 `Passive -> FixStand -> ArmPreAlign -> ArmLoco`
- action 解码为 `q_target = offset + raw_action * scale`，然后按关节限位裁剪
- raw action index `17` 会被锁成 `0.0`，对齐 real/mirror 里的 joint6 lock 设计

### 从 checkpoint 导出部署包

如果你训练出了新的 checkpoint，例如 `model_15000.pt`，先回到训练仓导出部署包：

```bash
cd "$B2ARX_TRAIN_ROOT"
conda activate isaaclab
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/rsl_rl/play_onnx.py \
  --task B2arx-v0 \
  --checkpoint "$B2ARX_TRAIN_ROOT/<run_dir>/model_15000.pt" \
  --export-deploy \
  --headless
```

导出完成后，同一个 run 目录下应该出现 / 更新：

```text
<run_dir>/exported/policy_full.onnx
<run_dir>/params/deploy.yaml
<run_dir>/exported/deploy_manifest.txt
```

`deploy_manifest.txt` 会记录这个 ONNX 是从哪个 checkpoint 导出的。这个信息也可以写进
`scripts/policy_deploy/deploy_config.example.yaml` 的 `policy.checkpoint` 和
`policy.manifest` 字段，方便启动时确认当前跑的是哪版策略。注意：这两个字段只是元数据；
实际推理加载的是 `policy.onnx`，或默认的 `<run_dir>/exported/policy_full.onnx`。

### 运行导出的策略

Nav2 模式不会使用通用的 `deploy_config.example.yaml`，而是自动切换到
[config/policies/basic_locomotion.yaml](config/policies/basic_locomotion.yaml)。该 profile 使用仓库内的
可移植基线 bundle：

```text
models/basic_locomotion/basic_locomotion_model.pt
models/basic_locomotion/exported/policy_full.onnx
models/basic_locomotion/params/deploy.yaml
models/basic_locomotion/bundle_manifest.txt
```

checkpoint 和 ONNX 由 Git LFS 管理，运行前执行 `git lfs pull`。运行时实际推理加载 ONNX，但保留
`basic_locomotion_model.pt` 作为训练来源和重新导出依据。换模型时不要只替换 `.pt`，必须同步更新 ONNX、
`deploy.yaml` 和 manifest。配置中的本地路径相对于 profile YAML 解析，因此 clone 到任意目录都可以使用。

创建新 profile 后，用下面的命令选择它：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --nav2 --deploy_config config/policies/<model-name>.yaml
```

`config/policies/` 管理模型和策略速度合同；
`ros_ws/src/b2arx_nav2_bringup/config/b2arx_nav2.yaml` 管理 planner、RPP、goal checker 与 costmap；
同目录中的 `zedx_nvblox_release_4_5.yaml`、`nvblox_*.yaml` 管理相机和建图参数。三类参数均可在不改
Python 的情况下调整。

启动时会在加载 ONNX 前校验 checkpoint、ONNX 和 deploy YAML 三项 SHA-256；当前已验证值为：

```text
checkpoint  dab9197db9ecec7c1496d23e548b02c1a7e22e17badbd075d5270f5cdb9630b4
ONNX        10b79e8531fdd1cb455d20c2079fe0c5b6dea6e9797dacf2061584e143c48f60
deploy YAML dac88692cf90dc173a25ae3144a7ded5ff583075c98666858fbbcf6b8fa652d6
```

上述参考 bundle 的运行合同是：73 维单帧 observation、30 帧历史（ONNX 输入 2190）、18 维 action、
`physics dt=0.005 s`、`policy dt=0.02 s`、action scale `0.25`、arm EMA `tau=0.02 s`。
`--nav2` 还会强制 `ArmLoco + ros2_twist`，订阅 `/cmd_vel` 与 `/cmd_vel_heartbeat`。

策略速度命令是 `base_link` 机体系的 `cmd=[vx, vy, wz]`；第三维 `wz` 是目标偏航角速度（rad/s），
不是绝对 yaw。策略 observation 中没有 absolute yaw 或 heading error，因此策略本身不能闭环修正航向；
Nav2 控制器必须根据路径方向在外部计算 `Twist.angular.z`。Nav2 入口还会把命令投影回训练域：
纯 `(vx=0, vy=0, wz!=0)` 命令全部置零，任何非零平移速度至少为 `0.25 m/s`，并限制
`|wz| <= min(0.6, planar_speed / 0.42)`。也就是 `wz` 的绝对上限为 `0.6 rad/s`，最小转弯半径保护为
`0.42 m`；这些值来自参考策略的训练范围和当前 Nav2 专用部署配置。替换模型时必须重新核对这些边界。

普通手柄/键盘 smoke test 仍可直接从 `ArmLoco` 启动；这条命令使用通用 example，
不等同于上面的 Nav2 reference bundle 合同：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --control_mode policy \
  --print_policy_debug
```

走完整自动 FSM 或自定义策略/速度/EE/输入设备：复制
`scripts/policy_deploy/deploy_config.example.yaml`，改 `deploy.start_state`、
`deploy.auto_arm_loco`、`deploy.command`、`deploy.ee_sphere`、`input.backend`
（scripted / keyboard / gamepad），再用 `--deploy_config <你的.yaml>` 指定：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --control_mode policy \
  --deploy_config /path/to/my_deploy_config.yaml \
  --duration 5.0 \
  --print_policy_debug
```

注意：`FixStand` 本身是 3 秒轨迹，`ArmPreAlign` 还需要 0.5 秒稳定门槛，所以完整自动流程至少要跑 3.6 秒以上。`--duration 0.2` 只能验证策略控制器和前置状态启动，不能证明已经进到 `ArmLoco`。

键盘遥控键位：`F`=FixStand `G`=ArmPreAlign `H`=ArmLoco `P`=Passive；
`R`=切换 EE 维度 `I`/`K`=当前维 ± `O`=EE 复位；方向键/小键盘走 vx/vy，`Z`/`X` 走 yaw。

手柄/Hitbox 按 MuJoCo mirror 语义：`A`=FixStand，左摇杆按键=ArmPreAlign，
`Y`/Start=ArmLoco，`B`=Passive，`X`=切换 EE 维度，Back/右摇杆按键=EE 复位；
D-pad 或左摇杆给 `vx/vy`，LB/RB 给 `wz`，LT/RT 是按住对当前 EE 维度做负/正向步进。
`--print_policy_debug` 会打印 `cmd=[vx vy wz]`、`ee_hold` 和 `ee_event`，方便确认输入已经进策略。

如果机械臂又开始抖，先不加载相机，只测默认辨识参数：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera
```

如果要排除桌子和物体的接触问题，只加载机器人和地面：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera --no_workspace
```

## 场景资产

默认场景故意保持比较简单，这样更容易调机器人稳定性、D455 几何位置和 EE target 转换。Isaac 官方资产不只在 `Isaac/Robots` 下面，找场景和物体时也可以看这些目录：

- `Isaac/Environments`
- `Isaac/Props`
- `Isaac/Props/YCB`

Asset Browser 里类似 `Thumbnail ... does not belong to file ...` 的 warning 通常是缩略图缓存 / 索引警告，不一定代表 USD 资产缺失。

直接加载官方背景场景：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --scene_asset warehouse
```

内置可选项：

- `--scene_asset minimal`：当前默认地面场景
- `--scene_asset grid`：`${ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd`
- `--scene_asset rough_plane`：`${ISAAC_NUCLEUS_DIR}/Environments/Terrains/rough_plane.usd`
- `--scene_asset warehouse`：`${ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd`
- `--scene_asset warehouse_local`：使用 `$B2ARX_WAREHOUSE_USD`，未设置时尝试
  `~/Documents/isaac-sim-assets-environments-5.1.0/.../warehouse.usd`
- `--scene_asset hospital`：`${ISAAC_NUCLEUS_DIR}/Environments/Hospital/hospital.usd`

也可以传本地路径、HTTP URL 或 Omniverse URL：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --environment_usd "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
```

对 B2+R5 操作项目来说，比较实用的视觉升级方式是先固定机器人 / 桌子 / 目标物几何，再往周围加官方道具，比如：

- `Props/PackingTable/packing_table.usd`
- `Props/Forklift/forklift.usd`
- `Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd`
- `Props/YCB/Axis_Aligned_Physics/*`
- `Environments/Simple_Warehouse/Props/*`

场景会持续打印机械臂诊断：

- `arm_abs_err_max`：机械臂关节相对目标的最大绝对误差
- `arm_abs_vel_max`：机械臂最大绝对关节速度
- `arm_abs_tau_max`：机械臂最大绝对输出力矩

桌子放在机器人前方，不和 B2 前腿重叠。如果 `--no_workspace` 稳定，但完整场景抖，问题大概率是桌子 / 物体接触或碰撞位置，而不是机械臂 PD 本身。

## 官方 ZED 仿真与 Isaac ROS Nvblox

仓库按下面的官方链路配置仿真与算法接口，而不是由 Isaac Sim 伪造 ZED 深度或 CameraInfo：

```text
official ZED_X.usdc
  -> sl.sensor.camera.ZED_Camera (ZED Camera Helper)
  -> ZED SDK simulation stream (IPC/network, port 30000)
  -> official zed_wrapper sim_mode
  -> official Isaac ROS Nvblox ZED remappings
```

必须明确：截至 Stereolabs Isaac Sim Extension `v4.3.0`，官方只支持 ZED X/X Mini/X Nano/X One，没有 ZED 2/2i 的 Isaac Sim USD 或 streamer model。因此仓库不再把自建 pinhole rig 称为“官方 ZED 2i”；严格走官方算法时，仿真型号必须使用 ZED X。ZED 2i 的官方 STL/xacro 只能提供外观和 TF，不能提供 ZED SDK 深度仿真。

官方依据：

- [Stereolabs：Using ZED with ROS 2 and Isaac Sim](https://docs.stereolabs.com/docs/integrations/isaac-sim/using-the-zed-with-ros-2-in-isaac-sim)
- [Stereolabs 官方扩展](https://github.com/stereolabs/zed-isaac-sim/releases/tag/v4.3.0)
- [NVIDIA Isaac Sim 5.1：ZED X certified by Stereolabs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_camera_depth_sensors.html#zed-x-certified-by-stereolabs)
- [Isaac ROS 4.5：Nvblox ZED tutorial](https://nvidia-isaac-ros.github.io/v/release-4.5/concepts/scene_reconstruction/nvblox/tutorials/tutorial_zed.html)

### 安装官方扩展

仓库固定使用支持 Isaac Sim 5.1/Kit 107.3 的 Stereolabs `v4.3.0`，安装器会下载官方 release 并校验 SHA-256：

```bash
python3 scripts/install_zed_isaac_sim.py
```

默认安装到 `~/.local/share/zed-isaac-sim/v4.3.0`；可用 `ZED_ISAAC_SIM_ROOT` 覆盖。仓库直接引用下载包内未经修改的 `ZED_X.usdc`，不会复制或重写相机、IMU、标定、碰撞体或算法。版本、checksum、资产路径和官方几何定义位于 [scripts/zed_isaac_sim.py](scripts/zed_isaac_sim.py)。

### 安装位姿

官方 USD 的 default prim `/Root` 保持原有 `RigidBodyAPI`，作为机器人 articulation 的 sibling 生成。场景在首次 `sim.reset()`/PLAY 前，使用 Isaac Sim 5.1 官方 `RobotAssembler.assemble_rigid_bodies()` 将 `/Root/base_link` 对齐到 `R5a/StereolabsZEDMount`，并创建 PhysX fixed joint；官方碰撞体通过 Robot Assembler 的 collision filtering 与机器人隔离，不删除任何 ZED 物理 API。最终 `/Root` 相对机械臂 `R5a` 使用用户给定的位姿：

```text
asset /Root translation = (+0.28, 0.0, -0.03) m
asset /Root rotation    = identity
```

`isaacsim.sensors.physics` 也由场景自动加入 Kit 启动参数，而不是在 Kit 启动后动态 enable。Isaac Sim 的 IMU manager 在启动阶段注册；加载过晚会导致官方 IMU 永远 `is_valid=false`，并阻断 Stereolabs helper 内部的 `IsaacReadIMU -> OgnZEDSimCameraNode` 执行链。

机器人坐标为 `+X` 前、`+Y` 左、`+Z` 上。官方 ZED X 相机朝资产 `+X`，baseline 为 `0.12 m`；其 USD 内部光心偏移为 `(0.015, ±0.06, 0.015)`，所以实际左右光心为：

```text
left  = (0.295, +0.06, -0.015) m in R5a
right = (0.295, -0.06, -0.015) m in R5a
```

查看相机并打印实际组合后的 USD 位姿：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --viewer_camera zed_left --print_zed_debug
```

### 启动官方 ZED SDK stream

场景默认启用官方 `ZED Camera Helper`，stream metadata 为 `HD1200 @ 30 Hz`，源码默认偶数端口为
`30000`、默认 transport 为官方 `NETWORK`。`--ros2` 只负责发布仿真 `/clock`；ZED 图像、深度、
CameraInfo、VIO、里程计和 ZED TF 全部由官方 `zed_wrapper 5.4` 产生。

当前仓库已经完成端到端验证，不再只是接口推演：

```text
official ZED X native streamer
  -> zed_wrapper 5.4 sim_mode (ULTRA, CUSTOM/2, 30 Hz, GEN_1)
  -> Isaac ROS Nvblox release-4.5
  -> nvblox::nav2::NvbloxCostmapLayer
  -> SmacPlanner2D + RegulatedPurePursuitController
  -> PositionGoalChecker (XY-only completion)
  -> velocity_smoother + cmd_vel_watchdog
  -> /cmd_vel + /cmd_vel_heartbeat
  -> cmd=[vx, vy, wz] -> configured locomotion policy -> B2 joint targets
```

这里的航向闭环由 Regulated Pure Pursuit（RPP）在策略外部完成：控制器沿路径计算
`wz = vx * path_curvature`，再通过 `Twist.angular.z` 送入策略。规划器使用
`SmacPlanner2D` 和 `use_final_approach_orientation=true`，只把实际路径的末段切线作为接近方向，
不把 RViz 目标箭头的 yaw 当成必须满足的终点约束。RPP 使用固定距离的普通路径 carrot：
`lookahead_dist=min_lookahead_dist=max_lookahead_dist=0.80 m`，并设置
`use_fixed_curvature_lookahead=false`、`interpolate_curvature_after_goal=false`。当前合同不使用旧实验中的
`1.70 m` fixed curvature lookahead，也不使用 SmacPlannerHybrid/Dubin 终点朝向搜索。

ZED 参数使用 `sensors_image_sync=true + pos_tracking_mode=GEN_1`，这是当前官方 local-stream/simulation
组合的同步 workaround；README 旧版写的 `STANDARD` 已不再使用。`debug.disable_nitros=true` 用于规避
部分 RTX 50/CUDA 13 环境中 zed_wrapper 5.4 NITROS publisher 的 CUDA 700，深度、VIO、Nvblox 和
Nav2 算法没有被替换。

Nvblox 使用官方 ZED wrapper topic 契约：

```text
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/camera_info
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/color/rect/camera_info
/zed/zed_node/pose
```

仓库的一体化 launch 只启动一个 `zed_wrapper`，不会再包含 Isaac ROS 教程中的第二个真机 driver；
Nvblox、两个 costmap 和全部 Nav2 lifecycle 节点都显式使用仿真时间。

### ZED、Nvblox、Nav2 双端启动

使用两个终端，并按“先 Isaac streamer，后 ROS receiver”的顺序启动。下面是已验证的参数结构；
Isaac 和 ROS 进程必须使用相同的 `ROS_DOMAIN_ID`、transport、IP 和端口。

终端 1，启动 Isaac Sim、ZED streamer 和配置文件指定的策略：

```bash
cd "$B2ARX_SIM_ROOT"
conda activate isaaclab

"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --nav2 \
  --ros2_domain_id 23 \
  --scene_asset warehouse \
  --no_workspace \
  --no_scene_camera \
  --viewer_camera scene \
  --print_policy_debug \
  --zed_stream_transport NETWORK \
  --zed_stream_port "$ZED_PORT"
```

等终端打印下面两类信息、Isaac 窗口中的机器人稳定站立后，再启动终端 2：

```text
Official Stereolabs ZED SDK stream graph active: ... port=$ZED_PORT transport=NETWORK
Policy source checkpoint metadata: .../checkpoint.pt
Policy bundle SHA256 verified: checkpoint=... onnx=... deploy=...
```

`NETWORK` 模式下，即使 producer 和 receiver 在同一台机器，也不要把 `sim_address` 写成
`127.0.0.1`。用 `ip -4 addr show scope global` 找 streamer 所在物理网卡地址，并在 ROS 端同步设置。

终端 2，装入本地 Isaac ROS/Nvblox、ZED SDK 和工作区 overlay，然后启动一体化 bringup：

```bash
cd "$B2ARX_SIM_ROOT"
source /opt/ros/jazzy/setup.bash

APT_ROOT="$ISAAC_ROS_ROOT/apt_root"
ZED_SDK_ROOT="$ISAAC_ROS_ROOT/zed_sdk_5_4"

export AMENT_PREFIX_PATH="$APT_ROOT/opt/ros/jazzy${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$APT_ROOT/opt/ros/jazzy${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PATH="$APT_ROOT/opt/ros/jazzy/bin:$PATH"
export LD_LIBRARY_PATH="$APT_ROOT/opt/ros/jazzy/lib:$APT_ROOT/opt/ros/jazzy/lib/x86_64-linux-gnu:$APT_ROOT/usr/lib/x86_64-linux-gnu:$ZED_SDK_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$APT_ROOT/opt/ros/jazzy/lib/python3.12/site-packages:$APT_ROOT/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export ZED_SDK_SETTINGS_PATH="$ZED_SDK_ROOT/settings/"
export ROS_DOMAIN_ID=23
export LC_ALL=C

source "$B2ARX_SIM_ROOT/ros_ws/install/local_setup.bash"

ZED_PORT="${ZED_PORT:-30000}"
ISAAC_HOST_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')"
test -n "$ISAAC_HOST_IP" || { echo "请手动设置 ISAAC_HOST_IP 为 Isaac Sim 的物理网卡 IPv4"; exit 1; }
ros2 launch b2arx_nav2_bringup b2arx_zed_nvblox_nav2.launch.py \
  domain_id:=23 \
  sim_address:="$ISAAC_HOST_IP" \
  sim_port:="$ZED_PORT" \
  use_rviz:=false \
  log_level:=info
```

端到端验收记录使用 `use_rviz:=false`；需要打开仓库预设的 Nvblox/Nav2 RViz 时可改成
`use_rviz:=true`。如果修改了 `ros_ws/src/b2arx_nav2_bringup` 下的 launch 或配置，启动前先执行：

```bash
cd "$B2ARX_SIM_ROOT/ros_ws"
colcon build --symlink-install --packages-select b2arx_nav2_bringup
source install/local_setup.bash
```

#### 端口和重启故障排查

源码默认端口是官方的 `30000`（SDK 会使用相邻配对端口）。端口可能被桌面 Omniverse 服务、旧的
Isaac 进程或其他 ZED 会话占用；不要把某台机器上的临时端口当成项目默认值。遇到下面的错误时，先查
端口而不是改相机或 Nvblox：

```text
[Streaming] Error: failed to create RTP Session (err:-74)
```

处理方法：

1. 退出已经报错的 Isaac Sim 实例，确认配置的端口确实释放：
   `ss -lunp | rg ":$ZED_PORT"`（也可用 `lsof -nP -iUDP:$ZED_PORT`）。
2. 选择一个未占用的**偶数**端口，把 Isaac 的 `--zed_stream_port` 和 ROS 的
   `sim_port:=` 改成同一个值；SDK 的配对端口会随之使用下一个奇数端口。
3. `NETWORK` 模式下将 `sim_address` 设置为 `ip -4 addr show scope global` 找到的物理网卡地址，
   不要写 `127.0.0.1`。
4. 不要为了修复端口错误杀掉 Omniverse Hub，也不要手动删除 `/dev/shm/sl_local_*`；先换端口并按
   “Isaac streamer → ROS receiver”的顺序重启。

如果 ROS bringup 单独中断超过 `5 s`，策略 watchdog 会让 Isaac 内 FSM 退回 `Passive`；仅重启 ROS
可能卡在 `ArmPreAlign`。日常测试应同时退出两个终端，再按上述顺序完整重启两端。

### 检查建图和 Nav2

保持终端 2 的环境，先确认 lifecycle、Nvblox map slice 和静止速度；三个 `topic hz` 命令逐条运行，
采样后按 `Ctrl-C` 再执行下一条：

```bash
for node in \
  /controller_server /smoother_server /planner_server \
  /behavior_server /bt_navigator /waypoint_follower \
  /velocity_smoother /local_costmap/local_costmap \
  /global_costmap/global_costmap
do
  ros2 lifecycle get "$node"
done

ros2 topic hz --wall-time --window 20 /nvblox_node/static_map_slice
ros2 topic hz --wall-time --window 20 /cmd_vel
ros2 topic hz --wall-time --window 20 /cmd_vel_heartbeat
ros2 topic echo /cmd_vel --once

ros2 param get /planner_server GridBased.plugin
ros2 param get /planner_server GridBased.use_final_approach_orientation
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server position_goal_checker.plugin
ros2 param get /controller_server FollowPath.lookahead_dist
ros2 param get /controller_server FollowPath.use_fixed_curvature_lookahead
ros2 param get /controller_server FollowPath.interpolate_curvature_after_goal
ros2 param get /controller_server FollowPath.use_rotate_to_heading
ros2 param get /velocity_smoother max_velocity
ros2 param get /velocity_smoother min_velocity
```

已验证基线结果为：Nav2 九个 lifecycle 节点全部 `active [3]`；Nvblox static map slice 为
`3.594 Hz`，ZED pose/odom 约 `9.4/9.8 Hz`；空闲时 `/cmd_vel` 为 `19.992 Hz` 且所有分量始终为零，
`/cmd_vel_heartbeat` 为 `19.999 Hz` 且序号持续递增。watchdog 在第一次 heartbeat 前永久保持零速度但
不让 FSM 退到 `Passive`；第一次 heartbeat 后仍保留 0.5 秒速度归零和 5 秒通信丢失进入 `Passive` 的保护。

参数查询应确认 `nav2_smac_planner::SmacPlanner2D`、`use_final_approach_orientation=true`、
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`、
`nav2_controller::PositionGoalChecker`、normal lookahead `0.80`、两个 curvature lookahead 开关均为 `false`、
`use_rotate_to_heading=false`，以及 velocity smoother 上下限 `[0.25, 0.0, 0.6]` / `[0.0, 0.0, -0.6]`。
导航过程中可在三个额外终端分别运行下面的命令，同时观察控制器原始输出、平滑器输出和 watchdog 最终
送往 Isaac Sim 的速度：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
```

Isaac Sim 终端的 `--print_policy_debug` 显示的是经过训练域保护后真正送入策略的
`cmd=[vx vy wz]`。该值允许全零或带平移的转弯，不应出现纯 `vx=0, vy=0, wz!=0`；非零平面速度
应至少为 `0.25 m/s`，且 `wz` 必须同时满足 `[-0.6, +0.6] rad/s` 和 `0.42 m` 最小转弯半径保护。

### RViz 看起来空旷但 costmap 有障碍

这是常见的显示层级误解，不一定表示深度相机故障。`Nvblox Mesh` 会显示地面、墙和所有重建表面；
Nav2 costmap 还会按 `inflation_radius=0.65 m` 膨胀占据单元。先分别关闭 RViz 的 `Nvblox Mesh`、
`Global Costmap` 和 `Local Costmap`，确认到底是哪一层在显示“障碍”。再检查 ZED 原始数据：

```bash
ros2 topic hz /zed/zed_node/depth/depth_registered
ros2 topic echo /zed/zed_node/depth/camera_info --once
ros2 topic hz /nvblox_node/static_map_slice
```

如果深度频率和 map slice 都正常，优先检查相机视野中的地面、仓库背景、机身/腿和机械臂；它们都可能
进入 Nvblox。`--no_workspace` 只移除桌子和测试物体，不会移除地面或背景。真正的空场景对照可用
`--scene_asset minimal --no_workspace`，并暂时只打开 costmap 图层，不要把 mesh 当成导航障碍图。

### 为什么旧配置会原地转圈

旧的 MPPI DiffDrive `FollowPath` 可以合法地产生 `vx=0, wz!=0`，其 `GoalAngleCritic` 还会继续追踪
RViz 2D Goal 箭头携带的绝对 yaw。`Twist.angular.z -> policy.wz` 的传递和符号本身没有问题；问题是
参考 locomotion policy 没有 absolute yaw observation，也没有纯原地旋转训练样本，所以无法完成这种终点航向动作。

当前配置改用 `SmacPlanner2D + RegulatedPurePursuitController + PositionGoalChecker`：RPP 只在向前沿路径
运动时提供外部 yaw-rate 修正，`use_rotate_to_heading=false` 禁止控制器先原地对向，终点只检查 XY。
即使上游意外发出纯旋转，策略入口也会把它安全地置零。中间实验使用过 `1.70 m` fixed curvature
lookahead，近侧向目标会持续保持同号曲率并弯成大环；最终配置恢复 `0.80 m` normal lookahead，关闭
fixed curvature lookahead 和 goal 后插值，这也是下面弧线目标实际通过的配置。

### 发送已验证的 NavigateToPose

参考 locomotion policy 没有纯原地旋转训练合同，因此 bringup 默认使用 Nav2 官方
`navigate_w_replanning_time.xml`，避免 recovery tree 调用 `Spin`。终点使用
`PositionGoalChecker`，没有 `yaw_goal_tolerance`：目标 quaternion 不参与完成判定。下面故意给目标设置
`yaw=pi`，用来验证机器人到达 XY 后不会继续原地追踪箭头朝向：

```bash
BT="$(ros2 pkg prefix nav2_bt_navigator)/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_time.xml"

ros2 action send_goal --feedback \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.25, y: -0.45, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}}}, behavior_tree: '${BT}'}"
```

这个近侧向、反向箭头目标在最终配置上已经实际返回 `Goal succeeded, error_code=0`。测试起点为
`(-0.500, 0.000, yaw=0)`，目标为 `(0.250, -0.450, yaw=pi)`；直线距离为 `0.875 m`，生成路径长度为
`0.912 m`，两者比值 `1.043`，没有 Dubins 式大环。弯行区间的 `wz` 实测为
`[-0.318, -0.237] rad/s`，没有任何 `vx=0,wz!=0` 样本；到达后动作返回成功，最终 XY 距离约
`0.25 m`，符合 `PositionGoalChecker` 的 `0.30 m` 容差，也没有继续追踪目标指定的 `yaw=pi`。

每次重置场景后仍应先在 RViz/Nvblox map 中确认示例坐标可通行；如果机器人起点已经变化，可以换一个
距离超过 `0.30 m` 的近侧向 XY，保留反向 quaternion 来复核同一合同。不要换回包含 `Spin` recovery
的 BT，除非重新训练并验证纯 `vx=0,wz!=0` 命令。

### 当前仿真限制

- 当前 RPP 合同只允许停止或以 `0.25 m/s` 向前沿弧线行走，不支持倒车和原地旋转；遇到必须原地调头才能
  通过的拓扑时，Nav2 应中止或重新规划，而不是向策略发送未训练命令。
- `PositionGoalChecker` 故意忽略 RViz 目标 quaternion。若后续任务要求底盘在终点达到指定绝对朝向，需增加
  独立的高层对齐动作或重新训练带 heading observation/纯旋转能力的策略，不能只恢复 `yaw_goal_tolerance`。
- ROS bringup 单独中断超过 5 秒会使仍在运行的策略 FSM 进入 `Passive`；当前再次建立 heartbeat 后的自动
  恢复可能卡在 `ArmPreAlign`。在恢复链路修复前，测试和日常使用都应完整重启 Isaac Sim 与 ROS 两端。
- `/zed/zed_node/odom` 当前的 `child_frame_id` 是 `zed_camera_link`，不是 `base_link`。相机位于底盘前方约
  `0.5 m`，转弯时相机点速度包含杆臂项。正式部署应增加把 ZED twist 转换到底盘参考点的 odometry adapter，
  或与机器人本体里程计融合；只修改消息中的 frame 字符串是错误的。
- wrapper 管理 `map -> odom -> zed_camera_link`，仓库再发布唯一的
  `zed_camera_link -> base_link` 安装边。不要再给 `zed_camera_link` 添加第二个 TF parent。

调试时可用 `--no_zed_sdk_stream` 只加载官方资产，或用 `--no_zed` 完全关闭 ZED。`--save_zed_frames` 只保存 Isaac Sim renderer 左右 RGB，明确不把 renderer ground-truth depth 当成 ZED SDK 输出。

## 回归验证

提交代码或修改 Nav2 配置后，先做不依赖 Isaac Sim 的离线检查，再做 ROS overlay 构建；这样可以尽早发现
命令合同、配置拼写和 Python 适配器回归：

```bash
cd "$B2ARX_SIM_ROOT"
conda activate isaaclab
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q  # 最近一次基线：82 passed
git diff --check

cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select b2arx_nav2_bringup
```

最后按[最小验收标准](#最小验收标准)做一次运行时检查，并用[已验证的 NavigateToPose](#发送已验证的-navigatetopose)
复核真实数据流。导航成功的最低判据是：Nvblox map slice 有持续消息、所有 Nav2 lifecycle 节点为 `active [3]`、
目标 action 返回 `error_code=0`、到达 XY 容差后 `/cmd_vel` 归零，而且策略调试输出不出现纯原地旋转命令。

## D455 相机

腕部相机现在使用 Isaac Sim 官方 RealSense D455 USD 资产，挂在 `R5a_link6/D455` 下面。场景通过 IsaacLab `CameraCfg(spawn=None)` 直接绑定官方 D455 asset 内部的 camera prim，而不是手动把相机塞进一个 mesh 里面。

当前 asset 中 camera prim 路径在：

```text
R5a_link6/D455/RSD455
```

相机流命名按 RealSense ROS 的习惯来：

- `scene["d455_color_camera"]`：彩色图，等价于 `/camera/color/image_raw`
- `scene["d455_depth_camera"]`：`distance_to_image_plane` 深度，等价于 `/camera/depth/image_rect_raw`
- `scene["d455_infra1_camera"]`：左红外风格图，等价于 `/camera/infra1/image_rect_raw`
- `scene["d455_infra2_camera"]`：右红外风格图，等价于 `/camera/infra2/image_rect_raw`

官方 D455 USD 路径会按当前 IsaacLab 使用的 Isaac 5.1 asset root 解析为：

```text
${ISAAC_NUCLEUS_DIR}/Sensors/Intel/RealSense/rsd455.usd
```

官方文档里对应的内部 camera prim 是：

- `Camera_Pseudo_Depth`
- `Camera_OmniVision_OV9782_Color`
- `Camera_OmniVision_OV9782_Left`
- `Camera_OmniVision_OV9782_Right`

默认 viewport 仍然看外部场景相机，这样能看到腕部 D455 本体。如果要切到腕部相机视角：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --viewer_camera color
```

`--viewer_camera depth` 只是把 viewport 切到 depth camera prim 的渲染 RGB 视角，不会把 Isaac viewport 变成深度 colormap。真正的深度数据来自 `distance_to_image_plane` tensor。

实时看深度伪彩色窗口：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --show_depth_preview
```

保存相机帧：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --save_camera_frames
```

打印 D455 asset 和 camera prim 路径：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py --print_d455_debug
```

保存的相机输出在 `outputs/camera`：

- `color_*.png`
- `infra1_*.png`
- `infra2_*.png`
- `depth_vis_*.png`：深度可视化图，只用于看效果
- `depth_m_*.npy`：米为单位的原始深度数组

## EE 目标转换

训练时的策略输入不是笛卡尔 EE 位置，而是以部署侧定义的球心为中心的球坐标目标。当前场景可以把选中的目标物世界坐标转换成 deploy-side EE sphere command。

打印选中物体对应的 EE 球坐标目标：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --print_ee_target_debug \
  --target_object red_box
```

短时间 headless 相机和 EE target smoke test：

```bash
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --headless \
  --duration 0.25 \
  --save_camera_frames \
  --print_ee_target_debug \
  --target_object red_box
```

EE sphere contract 记录在 `docs/d455_ee_target_pipeline.md`。

## D455 模型和安装位姿

当前场景会把官方 Isaac Sim RealSense D455 资产单独挂到腕部。旧的 D435i mesh 已经从 URDF 里移除，避免生成的机器人 USD 和官方 D455 重叠。

- D455 几何 helper：`scripts/d455_geometry.py`
- D455 asset 检查脚本：`scripts/inspect_d455_asset.py`
- 官方 D455 USD：`${ISAAC_NUCLEUS_DIR}/Sensors/Intel/RealSense/rsd455.usd`
- 生成后的机器人 USD：`assets/my_B2Arx/my_b2arx/my_robot.usd`

D455 相对 `R5a_link6` 的安装位姿保存在 `scripts/d455_geometry.py`：

- 平移：`(0.06, 0.0, 0.13)`
- 旋转：向下 pitch 30 度

IsaacLab 使用的深度 tensor 是 `distance_to_image_plane`；`depth_m_*.npy` 保存的就是米为单位的这个输出。Isaac viewport 不会自动把这个 tensor 显示成深度图，所以看深度请用 `--show_depth_preview` 或保存的 `depth_vis_*.png`。

## 重新生成机器人 USD

改了 URDF 或 mesh 后，用下面命令重新生成合并后的机器人 USD：

```bash
cd "$B2ARX_SIM_ROOT"
conda activate isaaclab
mkdir -p assets/my_B2Arx/my_b2arx
"$ISAACLAB_ROOT/isaaclab.sh" -p "$ISAACLAB_ROOT/scripts/tools/convert_urdf.py" \
  assets/my_B2Arx/my_robot.urdf \
  assets/my_B2Arx/my_b2arx/my_robot.usd \
  --merge-joints \
  --joint-target-type none \
  --headless
```

## 快速测试

只跑机器人和默认 hold 控制（关闭 D455 和 ZED X）：

```bash
cd "$B2ARX_SIM_ROOT"
conda activate isaaclab
"$ISAACLAB_ROOT/isaaclab.sh" -p scripts/isaac_b2arx_scene.py \
  --headless --duration 1.0 --no_scene_camera --no_zed
```

如果 Isaac 启动很慢，先查一下是不是有其它 Isaac 训练或仿真进程占着 GPU/CPU：

```bash
ps -ef | grep -E 'isaaclab|isaac_b2arx|fit_all_isaac' | grep -v grep
```

当前第一版场景包含：

- 从转换后 USD 加载的 B2 + ARX R5
- 地面
- 静态工作桌
- 三个用于感知 / 抓取测试的彩色刚体物体
- 挂在机械臂 `R5a` base 前方的官方 ZED X，经 Stereolabs extension 送入 ZED SDK 仿真流，可由 `zed_wrapper sim_mode` 接入
- 挂在 `R5a_link6` 上的官方 RealSense D455
- 可选的腕部 color / depth / infra 相机流
