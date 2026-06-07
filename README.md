# B2ARX Isaac Sim

这是 B2 机器狗 + ARX R5 机械臂的 Isaac Lab 仿真工作区，用来搭建具身操作场景、挂载 RealSense D455，并把训练好的策略重新部署回这个独立 Isaac 场景。

## 本地路径

- Isaac Lab：`/home/lbz/IsaacLab`
- Isaac 训练工作区：`/home/lbz/b2arx`
- 本项目工作区：`/home/lbz/b2arx_isaac_sim`
- 本地机器人资产：`assets/my_B2Arx`
- 默认机器人 USD：`assets/my_B2Arx/my_b2arx/my_robot.usd`
- 合并 URDF 源文件：`assets/my_B2Arx/my_robot.urdf`
- Conda 环境：`isaaclab`

## 启动场景

```bash
cd /home/lbz/b2arx_isaac_sim
conda activate isaaclab
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras
```

默认模式是 `hold`：每个物理步都会给机器人发送初始关节目标，让 B2+R5 不会因为重力直接塌下去。这个默认姿态和训练 / sim2sim 的初始设计对齐，机械臂 PD 默认使用最新的系统辨识纯 PD 参数。

- B2 站立姿态：FL/RL 为 `[+0.15, +0.67, -1.32]`，FR/RR 为 `[-0.15, +0.67, -1.32]`
- R5 机械臂姿态：`[0.0, 1.0, 0.8, 0.0, 0.0, 0.0]`
- 物理步长：`dt=0.005`，也就是 200 Hz
- 腿部 PD：hip/thigh `kp=300,kd=7.5`，calf `kp=500,kd=12.5`
- 机械臂默认 PD：`--arm_gain_profile identified`，来源是 `/home/lbz/arx_actuator_identification/arx_id_data/20260605_001412/fit_out/actuator_params_isaac.yaml`

如果想和训练时的高增益 arm profile 对比：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras --arm_gain_profile train
```

## 策略部署

场景现在可以不再只固定 demo 姿态，而是加载导出的 B2+R5 策略。部署链路对齐 `/home/lbz/b2arx/b2arx_sim2sim2real` 的实际控制设计：

- 使用嵌套的 `params/deploy.yaml`
- 使用 `exported/policy_full.onnx`
- 单帧 observation 是 73 维
- history 是 30 帧
- action 是 18 维：12 个 B2 腿部关节 + `R5a_joint1~6`
- FSM 为 `Passive -> FixStand -> ArmPreAlign -> ArmLoco`
- action 解码为 `q_target = offset + raw_action * scale`，然后按关节限位裁剪
- raw action index `17` 会被锁成 `0.0`，对齐 real/mirror 里的 joint6 lock 设计

默认策略包：

```text
/home/lbz/b2arx/b2arx_sim2real_v1/logs/rsl_rl/b2arx_direct/2026-06-07_02-01-02/exported/policy_full.onnx
/home/lbz/b2arx/b2arx_sim2real_v1/logs/rsl_rl/b2arx_direct/2026-06-07_02-01-02/params/deploy.yaml
```

直接从 `ArmLoco` 启动策略（默认 example 配置即为此场景）：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --control_mode policy \
  --print_policy_debug
```

走完整自动 FSM 或自定义策略/速度/EE/输入设备：复制
`scripts/policy_deploy/deploy_config.example.yaml`，改 `deploy.start_state`、
`deploy.auto_arm_loco`、`deploy.command`、`deploy.ee_sphere`、`input.backend`
（scripted / keyboard / gamepad），再用 `--deploy_config <你的.yaml>` 指定：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --control_mode policy \
  --deploy_config /path/to/my_deploy_config.yaml \
  --duration 5.0 \
  --print_policy_debug
```

注意：`FixStand` 本身是 3 秒轨迹，`ArmPreAlign` 还需要 0.5 秒稳定门槛，所以完整自动流程至少要跑 3.6 秒以上。`--duration 0.2` 只能验证策略控制器和前置状态启动，不能证明已经进到 `ArmLoco`。

键盘遥控键位：`F`=FixStand `G`=ArmPreAlign `H`=ArmLoco `P`=Passive；
`R`=切换 EE 维度 `I`/`K`=当前维 ± `O`=EE 复位；方向键/小键盘走 vx/vy，`Z`/`X` 走 yaw。

如果机械臂又开始抖，先不加载相机，只测默认辨识参数：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera
```

如果要排除桌子和物体的接触问题，只加载机器人和地面：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera --no_workspace
```

## 场景资产

默认场景故意保持比较简单，这样更容易调机器人稳定性、D455 几何位置和 EE target 转换。Isaac 官方资产不只在 `Isaac/Robots` 下面，找场景和物体时也可以看这些目录：

- `Isaac/Environments`
- `Isaac/Props`
- `Isaac/Props/YCB`

Asset Browser 里类似 `Thumbnail ... does not belong to file ...` 的 warning 通常是缩略图缓存 / 索引警告，不一定代表 USD 资产缺失。

直接加载官方背景场景：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --scene_asset warehouse
```

内置可选项：

- `--scene_asset minimal`：当前默认地面场景
- `--scene_asset grid`：`${ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd`
- `--scene_asset rough_plane`：`${ISAAC_NUCLEUS_DIR}/Environments/Terrains/rough_plane.usd`
- `--scene_asset warehouse`：`${ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd`
- `--scene_asset hospital`：`${ISAAC_NUCLEUS_DIR}/Environments/Hospital/hospital.usd`

也可以传本地路径、HTTP URL 或 Omniverse URL：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
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
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras --viewer_camera color
```

`--viewer_camera depth` 只是把 viewport 切到 depth camera prim 的渲染 RGB 视角，不会把 Isaac viewport 变成深度 colormap。真正的深度数据来自 `distance_to_image_plane` tensor。

实时看深度伪彩色窗口：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --show_depth_preview
```

保存相机帧：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --enable_cameras --save_camera_frames
```

打印 D455 asset 和 camera prim 路径：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --print_d455_debug
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
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --print_ee_target_debug \
  --target_object red_box
```

短时间 headless 相机和 EE target smoke test：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --headless \
  --enable_cameras \
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
cd /home/lbz/b2arx_isaac_sim
conda activate isaaclab
mkdir -p assets/my_B2Arx/my_b2arx
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p /home/lbz/IsaacLab/scripts/tools/convert_urdf.py \
  assets/my_B2Arx/my_robot.urdf \
  assets/my_B2Arx/my_b2arx/my_robot.usd \
  --merge-joints \
  --joint-target-type none \
  --headless
```

## 快速测试

只跑机器人和默认 hold 控制：

```bash
cd /home/lbz/b2arx_isaac_sim
conda activate isaaclab
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --duration 1.0 --no_scene_camera
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
- 挂在 `R5a_link6` 上的官方 RealSense D455
- 可选的腕部 color / depth / infra 相机流
