# B2ARX Policy FSM Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the exported B2+ARX R5 policy inside the D455 IsaacLab scene using the same deployment FSM and policy contract as `b2arx_sim2sim2real`.

**Architecture:** Add a small deployment package in `scripts/policy_deploy/` that mirrors the real/mirror runtime: nested `deploy.yaml` parsing, term-wise observation history, action decode, command buffer, and `Passive -> FixStand -> ArmPreAlign -> ArmLoco` FSM. The scene script becomes a host that reads Isaac articulation state, feeds the controller every deploy `step_dt`, and holds the latest joint targets between control ticks.

**Tech Stack:** Python 3, NumPy, PyYAML, ONNX Runtime in the `isaaclab` conda env, IsaacLab `Articulation` APIs.

---

### Task 1: Deployment Runtime Tests

**Files:**
- Create: `tests/test_policy_deploy.py`
- Create: `scripts/policy_deploy/runtime.py`
- Create: `scripts/policy_deploy/__init__.py`

- [ ] **Step 1: Write failing tests**

Add tests that define a tiny nested deploy config and verify:
- observation terms are assembled in deployment order and scaled term-wise,
- history is pre-filled and flattened,
- action 17 is locked before decode,
- decoded targets are clipped to joint limits,
- gait phase advances only when planar velocity norm is greater than `0.1`.

- [ ] **Step 2: Run tests and see import failure**

Run: `pytest -q tests/test_policy_deploy.py`
Expected: FAIL because `scripts.policy_deploy.runtime` does not exist.

- [ ] **Step 3: Implement `runtime.py`**

Implement:
- `gait_phase_train_semantics`
- `build_obs_termwise`
- `DeployPolicyRuntime`

Do not import `onnxruntime` unless an ONNX path is supplied.

- [ ] **Step 4: Run tests**

Run: `pytest -q tests/test_policy_deploy.py`
Expected: PASS.

### Task 2: FSM Controller

**Files:**
- Create: `scripts/policy_deploy/fsm.py`
- Modify: `tests/test_policy_deploy.py`

- [ ] **Step 1: Add failing FSM tests**

Verify:
- `Passive` outputs current joint positions,
- `FixStand` interpolates legs to stand target while holding arm position,
- `ArmPreAlign` gates ArmLoco until arm error stays below `0.05 rad` for `0.5 s`,
- `ArmLoco` enter order resets command buffer/history/raw action/arm smoother,
- `ArmLoco` locks raw action index 17 and applies arm EMA.

- [ ] **Step 2: Run tests and see missing FSM classes**

Run: `pytest -q tests/test_policy_deploy.py`
Expected: FAIL because FSM classes do not exist.

- [ ] **Step 3: Implement `fsm.py`**

Mirror `b2arx_sim2sim2real/b2arx_sim2sim/deploy/fsm` in pure Python without MuJoCo dependencies.

- [ ] **Step 4: Run tests**

Run: `pytest -q tests/test_policy_deploy.py`
Expected: PASS.

### Task 3: Isaac Scene Adapter

**Files:**
- Create: `scripts/policy_deploy/isaac_controller.py`
- Modify: `tests/test_policy_deploy.py`

- [ ] **Step 1: Add tests for deploy paths and default constants**

Verify default ONNX/YAML paths point at the newest exported deploy bundle and controlled joint names match the training order.

- [ ] **Step 2: Implement `isaac_controller.py`**

Implement:
- default deploy bundle paths,
- controlled joint names,
- quaternion projected gravity helpers,
- `IsaacDeployPlantAdapter` for reading Isaac robot state and writing targets/gains,
- `B2ArxIsaacPolicyController` for ticking FSM every deploy control step.

- [ ] **Step 3: Run tests**

Run: `pytest -q tests/test_policy_deploy.py`
Expected: PASS.

### Task 4: Scene CLI Integration

**Files:**
- Modify: `scripts/isaac_b2arx_scene.py`
- Modify: `README.md`

- [ ] **Step 1: Add CLI switches**

Add:
- `--control_mode {hold,policy}`
- `--policy_onnx`
- `--policy_deploy_yaml`
- `--policy_start_state`
- `--policy_command vx vy wz`
- `--policy_ee_sphere r pitch yaw`
- `--policy_auto_arm_loco`
- `--print_policy_debug`

- [ ] **Step 2: Use controller in `run_simulator`**

Keep current hold mode unchanged. In policy mode, build `B2ArxIsaacPolicyController`, call `reset()`, update on each physics step, and print FSM/action diagnostics.

- [ ] **Step 3: Update README commands**

Document policy mode command and explain it uses the same deploy FSM/contract as sim2sim2real.

### Task 5: Verification

**Files:**
- No new files.

- [ ] **Step 1: Unit tests**

Run: `source /home/lbz/miniforge3/etc/profile.d/conda.sh && conda activate isaaclab && pytest -q tests/test_policy_deploy.py`
Expected: PASS.

- [ ] **Step 2: Python compile**

Run: `source /home/lbz/miniforge3/etc/profile.d/conda.sh && conda activate isaaclab && python -m py_compile scripts/isaac_b2arx_scene.py scripts/policy_deploy/*.py`
Expected: exit 0.

- [ ] **Step 3: Isaac hold smoke**

Run: `TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --enable_cameras --duration 0.1 --control_mode hold`
Expected: scene starts, D455 loads, loop finishes.

- [ ] **Step 4: Isaac policy smoke**

Run: `TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py --headless --enable_cameras --duration 0.2 --control_mode policy --policy_auto_arm_loco --print_policy_debug`
Expected: controller loads deploy bundle, enters ArmLoco, prints finite action/target diagnostics, scene loop finishes.
