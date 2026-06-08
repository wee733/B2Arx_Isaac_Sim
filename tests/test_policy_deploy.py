from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest
import torch
import yaml

from scripts.policy_deploy.runtime import (
    DeployPolicyRuntime,
    build_obs_termwise,
    gait_phase_train_semantics,
)
from scripts.policy_deploy.fsm import (
    ArmLocoCommand,
    ArmLocoCommandBuffer,
    ArmLocoState,
    ArmPreAlignState,
    CtrlFSM,
    FixStandState,
    PassiveState,
)
from scripts.policy_deploy.isaac_controller import (
    CONTROLLED_JOINT_NAMES,
    DEFAULT_POLICY_DEPLOY_YAML,
    DEFAULT_POLICY_ONNX,
    _quat_wxyz_to_xyzw,
    controlled_joint_indices,
)


OBS_ORDER = [
    "projected_gravity_xy",
    "base_ang_vel",
    "joint_pos_rel",
    "joint_vel_rel",
    "last_action",
    "gait_phase",
    "velocity_commands",
    "ee_goal_sphere",
    "zero_force_obs",
]


POLICY_STIFFNESS = [300.0, 300.0, 500.0] * 4 + [1.0] * 6
POLICY_DAMPING = [7.5, 7.5, 12.5] * 4 + [2.0] * 6


def _deploy_dict(frame_stack: int = 3) -> dict:
    scales = {
        "projected_gravity_xy": [1.0, 1.0],
        "base_ang_vel": [0.25, 0.25, 0.25],
        "joint_pos_rel": [1.0] * 18,
        "joint_vel_rel": [0.05] * 18,
        "last_action": [1.0] * 18,
        "gait_phase": [1.0, 1.0],
        "velocity_commands": [2.0, 2.0, 0.25],
        "ee_goal_sphere": [0.5, 1.0, 1.3],
        "zero_force_obs": [1.0] * 6,
    }
    obs_history = {"use_gym_history": True}
    for name in OBS_ORDER:
        obs_history[name] = {
            "params": {"period": 0.64} if name == "gait_phase" else {},
            "clip": None,
            "scale": scales[name],
            "history_length": frame_stack,
        }
    return {
        "step_dt": 0.02,
        "stiffness": [float(i + 1) for i in range(18)],
        "damping": [float(i + 101) for i in range(18)],
        "actions": {
            "B2ArxJointPositionAction": {
                "joint_ids": None,
                "scale": [0.25] * 18,
                "offset": [float(i) for i in range(18)],
                "joint_lo": [float(i) - 0.2 for i in range(18)],
                "joint_hi": [float(i) + 0.2 for i in range(18)],
            }
        },
        "observations": {"obs_history": obs_history},
    }


def _write_deploy_yaml(tmp_path, frame_stack: int = 3):
    path = tmp_path / "deploy.yaml"
    path.write_text(yaml.safe_dump(_deploy_dict(frame_stack=frame_stack), sort_keys=False), encoding="utf-8")
    return path


def test_build_obs_termwise_uses_deploy_order_and_scales() -> None:
    deploy = _deploy_dict()
    obs_h = deploy["observations"]["obs_history"]
    obs_terms = [(name, obs_h[name]["scale"]) for name in OBS_ORDER]

    obs = build_obs_termwise(
        obs_terms,
        proj_grav_xy=np.array([0.1, -0.2], dtype=np.float32),
        base_ang_vel=np.array([1.0, 2.0, 3.0], dtype=np.float32),
        joint_pos_rel=np.ones(18, dtype=np.float32),
        joint_vel=np.full(18, 2.0, dtype=np.float32),
        last_action=np.full(18, 0.3, dtype=np.float32),
        gait_sincos=np.array([0.0, 1.0], dtype=np.float32),
        vel_cmd=np.array([0.5, -0.25, 0.4], dtype=np.float32),
        ee_sphere=np.array([0.36, 0.56, -0.2], dtype=np.float32),
    )

    assert obs.shape == (73,)
    assert np.allclose(obs[:5], [0.1, -0.2, 0.25, 0.5, 0.75])
    assert np.allclose(obs[23:41], np.full(18, 0.1))
    assert np.allclose(obs[61:64], [1.0, -0.5, 0.1])
    assert np.allclose(obs[64:67], [0.18, 0.56, -0.26])
    assert np.allclose(obs[67:73], np.zeros(6))


def test_policy_runtime_prefills_history_and_decodes_with_joint_limits(tmp_path) -> None:
    rt = DeployPolicyRuntime(_write_deploy_yaml(tmp_path, frame_stack=3))
    obs_a = np.arange(73, dtype=np.float32)
    obs_b = obs_a + 100.0

    rt.reset_history(obs_a)
    assert rt.obs_history_flat().shape == (219,)
    assert np.allclose(rt.obs_history_flat()[:73], obs_a)
    assert np.allclose(rt.obs_history_flat()[-73:], obs_a)

    rt.push(obs_b)
    flat = rt.obs_history_flat()
    assert np.allclose(flat[:73], obs_a)
    assert np.allclose(flat[-73:], obs_b)

    raw = np.zeros(18, dtype=np.float32)
    raw[0] = 10.0
    raw[1] = -10.0
    raw[17] = 4.0
    q = rt.decode(raw)
    assert q[0] == pytest.approx(0.2)
    assert q[1] == pytest.approx(0.8)
    assert q[17] == pytest.approx(17.2)


def test_policy_runtime_locks_joint6_action_before_decode(tmp_path) -> None:
    rt = DeployPolicyRuntime(_write_deploy_yaml(tmp_path, frame_stack=3))
    raw = np.zeros(18, dtype=np.float32)
    raw[16] = -0.5
    raw[17] = 3.0

    locked = rt.lock_action(raw)
    q = rt.decode(locked)

    assert locked[16] == pytest.approx(-0.5)
    assert locked[17] == pytest.approx(0.0)
    assert q[16] == pytest.approx(15.875)
    assert q[17] == pytest.approx(17.0)


def test_gait_phase_uses_training_planar_velocity_semantics() -> None:
    sincos, phase = gait_phase_train_semantics(0.0, 0.0, 0.0, 0.8, 0.02, 0.64)
    assert phase == pytest.approx(0.0)
    assert sincos == pytest.approx([0.0, 1.0])

    sincos, phase = gait_phase_train_semantics(0.0, 0.2, 0.0, 0.0, 0.02, 0.64)
    assert phase == pytest.approx(0.03125)
    assert sincos == pytest.approx([math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)])


@dataclass
class DummyState:
    q: np.ndarray
    dq: np.ndarray
    quat_xyzw: np.ndarray
    base_ang_vel: np.ndarray
    tilt_rad: float


class FakeRuntime(DeployPolicyRuntime):
    def __init__(self, deploy_yaml_path, raw_action):
        super().__init__(deploy_yaml_path)
        self.raw_action = np.asarray(raw_action, dtype=np.float32)
        self.infer_count = 0

    def infer(self) -> np.ndarray:
        self.infer_count += 1
        return self.raw_action.copy()


class FakePlant:
    def __init__(self, state: DummyState):
        self.state = state

    def read_state(self) -> DummyState:
        return self.state


class FakeCommandSource:
    def __init__(self, command: ArmLocoCommand):
        self.command = command

    def poll(self) -> ArmLocoCommand:
        return self.command


class FakeExplicitActuator:
    is_implicit_model = False

    def __init__(self, joint_indices, num_joints: int | None = None):
        self.joint_indices = np.asarray(joint_indices)
        count = len(joint_indices) if num_joints is None else int(num_joints)
        self.stiffness = torch.full((1, count), -1.0)
        self.damping = torch.full((1, count), -1.0)


class FakeImplicitActuator(FakeExplicitActuator):
    is_implicit_model = True


class FakeControllerPlant:
    device = "cpu"

    def __init__(self):
        self.written = []

    def write_control_gains(self, stiffness, damping, joint_ids=None) -> None:
        self.written.append((np.asarray(stiffness), np.asarray(damping), joint_ids))


def _state(q=None, dq=None, tilt_rad: float = 0.0) -> DummyState:
    return DummyState(
        q=np.asarray(q if q is not None else np.arange(18, dtype=np.float32), dtype=np.float32),
        dq=np.asarray(dq if dq is not None else np.zeros(18, dtype=np.float32), dtype=np.float32),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        tilt_rad=tilt_rad,
    )


def test_passive_and_fixstand_match_real_mirror_semantics() -> None:
    q0 = np.linspace(-0.2, 0.2, 18, dtype=np.float32)
    plant = FakePlant(_state(q=q0))

    passive = PassiveState()
    assert np.allclose(passive.run(plant, ArmLocoCommand()), q0)

    fixstand = FixStandState()
    fixstand.bind_plant(plant)
    fixstand.enter()
    out0 = fixstand.run(plant, ArmLocoCommand(), t=0.0)
    out3 = fixstand.run(plant, ArmLocoCommand(), t=3.0)

    assert np.allclose(out0[:12], q0[:12])
    assert np.allclose(out0[12:18], q0[12:18])
    assert np.allclose(out3[:12], FixStandState.QS2)
    assert np.allclose(out3[12:18], q0[12:18])


def test_arm_prealign_requires_stable_arm_before_loco(tmp_path) -> None:
    rt = DeployPolicyRuntime(_write_deploy_yaml(tmp_path))
    prealign = ArmPreAlignState(arm_target=rt.offset[12:18], leg_hold_target=FixStandState.QS2)

    prealign.enter()
    for _ in range(24):
        prealign.update_ready(arm_q=rt.offset[12:18] + 0.01, dt=0.02)
    assert not prealign.ready
    assert prealign.check_transition(ArmLocoCommand(arm_loco_pressed=True)) is None

    prealign.update_ready(arm_q=rt.offset[12:18] + 0.01, dt=0.02)
    assert prealign.ready
    assert prealign.check_transition(ArmLocoCommand(arm_loco_pressed=True)) == "ArmLoco"


def test_arm_loco_enter_and_run_match_deploy_fsm(tmp_path) -> None:
    raw = np.zeros(18, dtype=np.float32)
    raw[0] = 0.4
    raw[16] = 1.0
    raw[17] = 5.0
    rt = FakeRuntime(_write_deploy_yaml(tmp_path), raw_action=raw)
    buf = ArmLocoCommandBuffer()
    plant = FakePlant(_state(q=rt.offset.copy()))
    loco = ArmLocoState(rt, buf, control_dt=0.02, arm_ema_tau=0.02)

    loco.enter(plant)
    assert rt.obs_history_flat().shape == (219,)
    assert np.allclose(loco.last_raw_action, np.zeros(18))
    assert np.allclose(loco.arm_smooth, rt.offset[12:18])

    q_target = loco.run(plant, ArmLocoCommand(vx=0.2))

    assert rt.infer_count == 1
    assert loco.last_raw_action[0] == pytest.approx(0.4)
    assert loco.last_raw_action[16] == pytest.approx(1.0)
    assert loco.last_raw_action[17] == pytest.approx(0.0)
    assert q_target[0] == pytest.approx(0.1)
    expected_joint5 = rt.joint_hi[16]
    alpha = 1.0 - math.exp(-1.0)
    assert q_target[16] == pytest.approx(rt.offset[16] + alpha * (expected_joint5 - rt.offset[16]))
    assert q_target[17] == pytest.approx(rt.offset[17])


def test_ctrl_fsm_shared_safety_and_transition_order(tmp_path) -> None:
    rt = DeployPolicyRuntime(_write_deploy_yaml(tmp_path))
    plant = FakePlant(_state(q=np.zeros(18, dtype=np.float32), tilt_rad=0.0))
    passive = PassiveState()
    fixstand = FixStandState()
    fixstand.bind_plant(plant)
    prealign = ArmPreAlignState(arm_target=rt.offset[12:18], leg_hold_target=FixStandState.QS2)
    loco = ArmLocoState(rt, ArmLocoCommandBuffer(), control_dt=0.02)
    fsm = CtrlFSM(
        {
            "Passive": passive,
            "FixStand": fixstand,
            "ArmPreAlign": prealign,
            "ArmLoco": loco,
        },
        initial="Passive",
    )

    fsm.tick(plant, ArmLocoCommand(fixstand_pressed=True), tilt_rad=0.0)
    assert fsm.current_name == "FixStand"

    fsm.tick(plant, ArmLocoCommand(passive_pressed=True), tilt_rad=0.0)
    assert fsm.current_name == "Passive"

    fsm.tick(plant, ArmLocoCommand(fixstand_pressed=True), tilt_rad=1.2)
    assert fsm.current_name == "Passive"


def test_ctrl_fsm_can_start_directly_in_arm_loco_with_plant(tmp_path) -> None:
    rt = DeployPolicyRuntime(_write_deploy_yaml(tmp_path))
    plant = FakePlant(_state(q=rt.offset.copy()))
    loco = ArmLocoState(rt, ArmLocoCommandBuffer(), control_dt=0.02)

    fsm = CtrlFSM({"ArmLoco": loco}, initial="ArmLoco", plant=plant)

    assert fsm.current_name == "ArmLoco"
    assert rt.obs_history_flat().shape == (219,)
    assert np.allclose(loco.last_raw_action, np.zeros(18))
    assert np.allclose(loco.arm_smooth, rt.offset[12:18])


def test_command_buffer_pinned_init_survives_reset() -> None:
    # Mirrors the controller fix: pinning INIT to a configured sphere means
    # ArmLocoState.enter()'s reset_to_init() restores it, not the hardcoded default.
    buf = ArmLocoCommandBuffer()
    buf.set([0.5, 0.3, -0.4])
    buf.INIT = tuple(buf.get())
    buf.cycle_selected_dimension()
    buf.step_selected_dimension(1)   # perturb away from the configured sphere
    buf.reset_to_init()
    assert buf.get() == pytest.approx([0.5, 0.3, -0.4])


def test_controller_records_last_command_after_auto_overlay() -> None:
    from scripts.policy_deploy.isaac_controller import B2ArxIsaacPolicyController

    source_cmd = ArmLocoCommand(vx=0.2)
    controller = B2ArxIsaacPolicyController.__new__(B2ArxIsaacPolicyController)
    controller.command_source = FakeCommandSource(source_cmd)
    controller.auto_arm_loco = True
    controller.fsm = SimpleNamespace(current_name="Passive", states={})

    cmd = controller._command_for_current_state()

    assert cmd.fixstand_pressed is True
    assert controller.last_command.vx == pytest.approx(0.2)
    assert controller.last_command.fixstand_pressed is True
    cmd.vx = 9.0
    assert controller.last_command.vx == pytest.approx(0.2)


def test_controller_state_gains_update_explicit_actuator_model() -> None:
    from scripts.policy_deploy.isaac_controller import B2ArxIsaacPolicyController

    controller = B2ArxIsaacPolicyController.__new__(B2ArxIsaacPolicyController)
    controller.runtime = SimpleNamespace(
        stiffness=np.asarray(POLICY_STIFFNESS, dtype=np.float32),
        damping=np.asarray(POLICY_DAMPING, dtype=np.float32),
    )
    controller.fsm = SimpleNamespace(current_name="Passive")
    controller.plant = FakeControllerPlant()
    controller.plant.joint_ids = list(range(18))
    controller.robot = SimpleNamespace(
        num_joints=20,
        actuators={
            "legs": FakeExplicitActuator([0, 3, 6]),
            "arm": FakeExplicitActuator([12, 13, 14, 15, 16, 17]),
            "gripper": FakeExplicitActuator([18, 19]),
        }
    )
    controller._last_kp = np.full(18, -1.0, dtype=np.float32)
    controller._last_kd = np.full(18, -1.0, dtype=np.float32)

    controller._apply_gains_for_state()

    assert controller.robot.actuators["legs"].stiffness.tolist() == [[0.0, 0.0, 0.0]]
    assert controller.robot.actuators["legs"].damping.tolist() == [[10.0, 10.0, 10.0]]
    assert controller.robot.actuators["arm"].stiffness.tolist() == [[0.0] * 6]
    assert controller.robot.actuators["arm"].damping.tolist() == [[10.0] * 6]
    assert controller.robot.actuators["gripper"].stiffness.tolist() == [[-1.0, -1.0]]
    assert controller.robot.actuators["gripper"].damping.tolist() == [[-1.0, -1.0]]
    assert controller.plant.written == []


def test_controller_state_gains_update_implicit_actuator_drive_only_for_controlled_joints() -> None:
    from scripts.policy_deploy.isaac_controller import B2ArxIsaacPolicyController

    controller = B2ArxIsaacPolicyController.__new__(B2ArxIsaacPolicyController)
    controller.runtime = SimpleNamespace(
        stiffness=np.asarray(POLICY_STIFFNESS, dtype=np.float32),
        damping=np.asarray(POLICY_DAMPING, dtype=np.float32),
    )
    controller.fsm = SimpleNamespace(current_name="ArmLoco")
    controller.plant = FakeControllerPlant()
    controller.plant.joint_ids = list(range(18))
    controller.robot = SimpleNamespace(
        num_joints=20,
        actuators={
            "implicit_hips": FakeImplicitActuator([0, 3, 6, 9]),
            "explicit_arm": FakeExplicitActuator([12, 13, 14, 15, 16, 17]),
            "gripper": FakeExplicitActuator([18, 19]),
        },
    )
    controller._last_kp = np.full(18, -1.0, dtype=np.float32)
    controller._last_kd = np.full(18, -1.0, dtype=np.float32)

    controller._apply_gains_for_state()

    assert len(controller.plant.written) == 1
    stiffness, damping, joint_ids = controller.plant.written[0]
    assert stiffness.tolist() == [[300.0, 300.0, 300.0, 300.0]]
    assert damping.tolist() == [[7.5, 7.5, 7.5, 7.5]]
    assert joint_ids == [0, 3, 6, 9]
    assert controller.robot.actuators["implicit_hips"].stiffness.tolist() == [[300.0, 300.0, 300.0, 300.0]]
    assert controller.robot.actuators["implicit_hips"].damping.tolist() == [[7.5, 7.5, 7.5, 7.5]]
    assert controller.robot.actuators["explicit_arm"].stiffness.tolist() == [[1.0] * 6]
    assert controller.robot.actuators["explicit_arm"].damping.tolist() == [[2.0] * 6]


def test_arm_loco_command_has_only_mirror_ee_fields() -> None:
    cmd = ArmLocoCommand()

    assert not hasattr(cmd, "ee_pitch_step")
    assert not hasattr(cmd, "ee_yaw_step")


def test_command_buffer_steps_selected_dimension_like_mirror() -> None:
    buf = ArmLocoCommandBuffer()

    buf.step_selected_dimension(1)
    assert buf.get() == pytest.approx([0.38, 0.56, 0.0])

    buf.cycle_selected_dimension()
    buf.step_selected_dimension(1)
    assert buf.get() == pytest.approx([0.38, 0.61, 0.0])

    buf.cycle_selected_dimension()
    buf.step_selected_dimension(-1)
    assert buf.get() == pytest.approx([0.38, 0.61, -0.05])


def test_isaac_controller_defaults_and_joint_order() -> None:
    assert DEFAULT_POLICY_ONNX.name == "policy_full.onnx"
    assert DEFAULT_POLICY_DEPLOY_YAML.name == "deploy.yaml"
    assert DEFAULT_POLICY_ONNX.is_file()
    assert DEFAULT_POLICY_DEPLOY_YAML.is_file()
    assert CONTROLLED_JOINT_NAMES[:3] == (
        "b2_description_FL_hip_joint",
        "b2_description_FL_thigh_joint",
        "b2_description_FL_calf_joint",
    )
    assert CONTROLLED_JOINT_NAMES[12:18] == (
        "R5a_joint1",
        "R5a_joint2",
        "R5a_joint3",
        "R5a_joint4",
        "R5a_joint5",
        "R5a_joint6",
    )


def test_controlled_joint_indices_preserve_policy_order() -> None:
    joint_names = ["unused", *CONTROLLED_JOINT_NAMES[:4], "another", *CONTROLLED_JOINT_NAMES[4:]]
    ids = controlled_joint_indices(joint_names)
    assert ids == [joint_names.index(name) for name in CONTROLLED_JOINT_NAMES]


def test_quaternion_conversion_wxyz_to_xyzw() -> None:
    assert np.allclose(_quat_wxyz_to_xyzw(np.array([1.0, 2.0, 3.0, 4.0])), [2.0, 3.0, 4.0, 1.0])
