# 部署命令输入层设计 (Deploy Command-Input Layer)

> Spec for wiring live keyboard/gamepad input into the B2+ARX R5 deployment FSM,
> unifying configuration under a single `deploy_config.yaml`.

## 目标

把活的输入设备(键盘、手柄、脚本)统一成一个 `ArmLocoCommand`,喂给已稳定的部署
FSM,使策略能在 IsaacLab 场景里被实时遥控。配置入口收敛到单个
`deploy_config.yaml`。**不改动**已通过测试的 `runtime.py` / `fsm.py`——策略部署契约已定型。

非目标(本期不做):
- 不接 `SceneConfig`(environment_usd / robot_usd);场景资产与控制输入是两摊事。
- 不接 ROS2 / Isaac ROS / 远端 command;但架构必须为它们留好同一个 `CommandSource` 接口。
- 不实现 EE 长按重复步进(real-mirror 的 0.12s held-repeat);v1 只做 one-shot,明确简化。

## 与官方 PolicyController 的关系

官方 `isaacsim.robot.policy.examples.PolicyController`(本机
`.../controllers/policy_controller.py`)是一个薄基类:`load_policy` 用
`torch.jit.load` 读 `.pt` + 从 env.yaml 取 decimation/dt/增益;子类实现
`_compute_observation(command)` 手搓 obs;`forward(dt, command)` 靠
`_policy_counter % decimation` 降频跑策略,`action_scale*action + default_pos`
当位置目标。Spot 例子的 `command` 是 `(vx, vy, wz)`,由调用方传入。

本仓库的 `B2ArxIsaacPolicyController` + `runtime.py` 是该模式的**超集**:多了
ONNX、term-wise obs history、四态 FSM、EE 球坐标命令。官方教程同样没解决"command
从哪来"——本 spec 补的就是这一层。用户的"play 脚本 env=1 就是部署"直觉成立:
本质是把训练 obs/act 管线在单环境跑一遍,但 obs 必须在部署侧手写重建(官方与本仓库
皆如此),因为部署没有 env manager。

## 架构边界

```
设备(carb 事件线程)            控制拍 step_dt (~50Hz)
─────────────────            ──────────────────────────────────
键盘 KEY_PRESS ─add_callback─▶ _CommandLatch.set(...)
手柄按钮 carb event ─▶ ButtonEdgeFilter ─rising─▶ _CommandLatch.set(...)
摇杆/方向键 ─▶ Se2*.advance() 缓存 vx/vy/wz
                                  │
                                  ▼ controller.update(sim_dt) 累加到 step_dt
                          CommandSource.poll() -> ArmLocoCommand
                            cmd.vx/vy/wz = Se2*.advance()        # 连续,按住持续
                            合并 latch 边沿事件(状态键/EE)       # 一次性,读后清零
                                  │
                                  ▼
                          CtrlFSM.tick(plant, cmd, tilt) -> q_target -> apply_targets
```

未来把 `CommandSource` 换成 ROS2 topic / Isaac ROS 视觉 / 远端设备时,controller
与 FSM 完全不感知来源——只消费 `ArmLocoCommand`。这是本设计服务算法的核心价值。

## 组件

新增子包 `scripts/policy_deploy/command_sources/`(改名:不用泛泛的 `input/`,避免与
Python `input()`、ROS input 概念混淆;现状只是 docstring 桩,改名零成本)。

| 文件 | 单元 | 职责 | 依赖 | 可测 |
|---|---|---|---|---|
| `base.py` | `CommandSource(ABC)` | `poll()->ArmLocoCommand`; `reset()`; `is_stale(now_s=None)->bool`(默认 False); `close()`(默认 no-op) | 无 | — |
| `base.py` | `ScriptedCommandSource` | 吐固定 vx/vy/wz(配置来) | 无 | 纯 Python |
| `latch.py` | `_CommandLatch` | 暂存一次性边沿事件,poll 读出后清零 | 无 | 纯 Python |
| `edge.py` | `ButtonEdgeFilter` | `update(name, value)->bool` 上升沿判定 | 无 | 纯 Python |
| `devices.py` | `KeyboardCommandSource` | 包 `Se2Keyboard`:advance 取速度 + add_callback 绑状态键 | carb/omni | smoke |
| `devices.py` | `GamepadCommandSource` | 包 `Se2Gamepad`:advance 取速度 + 自订阅 carb 按钮事件 | carb/omni | smoke |
| `__init__.py` | `make_command_source(input_settings, deploy_settings)` | 工厂,路由 keyboard/gamepad/scripted | 延迟导入 carb | 纯 Python(scripted 路径) |

**关键约束**:`Se2Keyboard` / `Se2Gamepad` 依赖 carb/omni,必须在 `simulation_app`
启动后构造。所以 `devices.py` 内**延迟导入** carb——`command_sources` 包能在纯
pytest 里 import,只有真正构造键盘/手柄源时才碰 carb。`ScriptedCommandSource` 无此
限制。

## 职责划分要点

- **连续量**(vx/vy/wz):每拍重新读 `Se2*.advance()`,按住持续有效。
- **离散量**(状态切换、EE 调节):边沿触发,latch 攒着,`poll()` 读完即清零。
  **同类事件在同一控制拍内合并、不做计数;v1 只保证"至少触发一次"**(状态切换、
  EE one-shot 都能接受)。边沿质量由设备适配层各自保证(键盘 KEY_PRESS 天生干净,
  手柄靠 `ButtonEdgeFilter`),latch 本身不做阈值/debounce/计数。
- **staleness**:`CommandSource.is_stale(now_s)` 默认 `False`;键盘/手柄/scripted 本地
  源永不 stale。controller 每拍 `stale = source.is_stale()` 传给 `fsm.tick(..., stale=
  stale)`——`CtrlFSM` 已有该入参(stale 时回 Passive)。这是给未来 ROS2/Thor 网络命令
  留的 watchdog 口子,本期不实现具体超时逻辑。
- **资源释放**:`CommandSource.close()` 默认 no-op;`GamepadCommandSource` 在 `close()`
  里 `unsubscribe_to_gamepad_events` 退订自己那份 carb 订阅(`__del__` 兜底),避免重启
  场景/退出时残留回调。controller 销毁或场景收尾时调 `source.close()`。
- **auto_arm_loco** 留在 controller(依赖 FSM 状态 / `_state_elapsed` /
  `ArmPreAlign.ready`),**不进** command source。`ScriptedCommandSource` 只吐固定命令。

## 键位映射

**键盘**(`Se2Keyboard.add_callback`,只在 KEY_PRESS 触发,干净边沿)。避开官方已占用的
键:`L`=reset、`Z/X`=yaw、方向键/Numpad=速度。状态与 EE 用空闲键:

| 键 | 事件 | | 键 | 事件 |
|---|---|---|---|---|
| `F` | FixStand | | `R` | EE 切换维度 |
| `G` | ArmPreAlign | | `I` | EE 当前维 + |
| `H` | ArmLoco | | `K` | EE 当前维 − |
| `P` | Passive | | `O` | EE reset |

速度沿用官方:方向键 / Numpad 8/2/4/6 走 `vx/vy`,`Z/X` 走 `wz`。

**手柄**(连续速度用 `Se2Gamepad.advance()`;按钮**自订阅** `carb.input
.subscribe_to_gamepad_events`,**不用** `Se2Gamepad.add_callback()`——后者触发时
`self._additional_callbacks[event.input]()` 无参,拿不到 `event.value`,无法判边沿)。
按钮经 `ButtonEdgeFilter` 上升沿过滤(`PRESS_THRESH=0.5`),语义沿用 real-mirror:

`A`=FixStand `LeftThumb`=ArmPreAlign `Y`/`Start`=ArmLoco `B`=Passive
`X`=EE 切维 `Back`=EE reset;左摇杆=vx/vy,右摇杆=wz,扳机=EE ±(one-shot)。

`GamepadCommandSource` 自持 carb 订阅句柄,`close()` 里 `unsubscribe_to_gamepad_events`
退订(`__del__` 兜底)。注意同一 gamepad 会有两份 carb 订阅:`Se2Gamepad` 内部一份管
摇杆轴,本类一份管按钮;carb 支持多订阅者,冒烟时实测确认不打架。

## 配置接线

`deploy_config.py` 三层结构(`PolicyConfig`/`SceneConfig`/`DeploySettings`/
`DeployConfig`)已存在,新增一个 `input:` 段:

```yaml
policy:
  run_dir: /home/lbz/b2arx/b2arx_sim2real_v1/logs/rsl_rl/b2arx_direct/2026-06-07_02-01-02
  # onnx / deploy_yaml: 可选覆盖,默认走 run_dir/exported|params 标准布局

deploy:
  start_state: ArmLoco        # Passive / FixStand / ArmPreAlign / ArmLoco
  auto_arm_loco: false
  ee_sphere: [0.36, 0.56, 0.0]
  command: [0.0, 0.0, 0.0]    # scripted backend 的固定 vx/vy/wz

input:
  backend: scripted           # scripted / keyboard / gamepad
  keyboard:
    v_x_sensitivity: 0.8
    v_y_sensitivity: 0.4
    omega_z_sensitivity: 1.0
  gamepad:
    v_x_sensitivity: 1.0
    v_y_sensitivity: 1.0
    omega_z_sensitivity: 1.0
    dead_zone: 0.01
```

字段名与 IsaacLab `Se2KeyboardCfg`/`Se2GamepadCfg` 对齐(已核源码)。`input:` 解析进
新增的 `InputSettings` dataclass(含 `backend` + `keyboard`/`gamepad` 灵敏度子字典),
作为 `DeployConfig` 的第四个字段 `input`;现有 `DeploySettings.input_backend` 这个
扁平字段废弃,迁移到 `InputSettings.backend`。这套配置只存在于手写的 `deploy_config
.yaml`,不污染训练导出的 `params/deploy.yaml`。

**CLI**:只走 yaml。保留单个 `--deploy_config <path>`,默认指向仓库内真实可跑的
`scripts/policy_deploy/deploy_config.example.yaml`;**删除** `--policy_onnx`/
`--policy_deploy_yaml`/`--policy_start_state`/`--policy_command`/`--policy_ee_sphere`/
`--policy_auto_arm_loco`。`make_policy_controller` 改为读 cfg → 解析 onnx/deploy_yaml →
`make_command_source(cfg.input, cfg.deploy)` → 构造 controller。

**controller 改动(最小侵入)**:构造参数 `command: ArmLocoCommand` → `command_source:
CommandSource`;`_command_for_current_state()` 里 `cmd = self.command_source.poll()`
取代复制静态命令,auto_arm_loco 叠加逻辑原样保留。FSM / runtime / EE buffer 不动。

## 错误处理

- `--deploy_config` 默认 example 必须真实可跑(非纯注释模板);缺失 → `load_deploy_config`
  抛 `FileNotFoundError`(已实现)。
- `backend` 非法 → 工厂抛 `ValueError`,列出合法值。
- 手柄请求但 `get_gamepad(0)` 无设备 → 打印
  `[WARN] gamepad unavailable, falling back to scripted command source`,回退 scripted,不崩。
- carb/omni 导入失败(非 Isaac 环境)→ 仅在构造键盘/手柄源时才会发生;`command_sources`
  包导入与 scripted 路径不触发。

## 测试(纯 Python,`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`)

- `_CommandLatch`:set → poll 读出 → 自动清零;同拍多次 set 只吐一次;poll 后再读为空。
- `ButtonEdgeFilter`:0→1 触发一次;1→1 不重复;1→0 不触发;低于阈值视作释放。
- `ScriptedCommandSource`:poll 返回配置的 vx/vy/wz;不掺 auto_arm_loco;`is_stale()`
  恒 False、`close()` 不报错。
- `make_command_source`:scripted 路由正确且不 import carb;非法 backend 报错。
- `deploy_config.py`:解析 `input:` 段(backend + 灵敏度),缺省回落默认值。
- 键盘/手柄源构造依赖 carb,**不进**单测,靠 Isaac 冒烟覆盖。

## 冒烟验证(Isaac,headless)

1. **direct ArmLoco**:example 设 `start_state: ArmLoco` + `backend: scripted`,
   `--duration 1.05`(诊断每 1.0s 打一次,需跨过首次打印才看得到非零 raw action),
   确认 `state == ArmLoco`、ONNX 动作有限、循环正常收尾。
   (若只想验证"加载+进入 ArmLoco+不崩",`--duration 0.2` 即可,但那时 raw action 仍为 0,
   不能据此判断动作输出。)
2. **full FSM**:`start_state: Passive` + `auto_arm_loco: true`,`--duration 5.0`
   (FixStand 需 3s),确认最终进 ArmLoco。
3. **hold 回归**:`--control_mode hold --duration 0.1` 不受影响。

(键盘/手柄需有头窗口与真实设备,手动验证,不入自动冒烟。)

## 构建顺序

**前置**:工作树非干净(README/assets/scripts/tests 有已存在的修改与未跟踪文件,见
`git status`)。这些是先前工作的产物,实现时**在当前工作区基础上继续,不回滚**。

1. `command_sources/{base,latch,edge}.py` + 单测(纯逻辑,先 TDD)。
2. `deploy_config.py` 加 `input:` 段 + 单测。
3. `command_sources/devices.py` + `__init__.py` 工厂(carb 延迟导入)。
4. `isaac_controller.py`:`command` → `command_source` 接线;每拍传 `stale=source.is_stale()`。
5. `isaac_b2arx_scene.py`:CLI 收敛到 `--deploy_config`,改 `make_policy_controller`;
   场景收尾调 `source.close()`。
6. `deploy_config.example.yaml` + README 更新(同步 smoke 命令)。
7. 冒烟验证。
