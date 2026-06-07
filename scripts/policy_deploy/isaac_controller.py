from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from .fsm import (
    ArmLocoCommand,
    ArmLocoCommandBuffer,
    ArmLocoState,
    ArmPreAlignState,
    CtrlFSM,
    FixStandState,
    PassiveState,
)
from .runtime import DeployPolicyRuntime
from .command_sources import CommandSource, ScriptedCommandSource


DEFAULT_POLICY_RUN_DIR = Path(
    "/home/lbz/b2arx/b2arx_sim2real_v1/logs/rsl_rl/b2arx_direct/2026-06-07_02-01-02"
)
DEFAULT_POLICY_ONNX = DEFAULT_POLICY_RUN_DIR / "exported" / "policy_full.onnx"
DEFAULT_POLICY_DEPLOY_YAML = DEFAULT_POLICY_RUN_DIR / "params" / "deploy.yaml"

CONTROLLED_JOINT_NAMES = (
    "b2_description_FL_hip_joint",
    "b2_description_FL_thigh_joint",
    "b2_description_FL_calf_joint",
    "b2_description_FR_hip_joint",
    "b2_description_FR_thigh_joint",
    "b2_description_FR_calf_joint",
    "b2_description_RL_hip_joint",
    "b2_description_RL_thigh_joint",
    "b2_description_RL_calf_joint",
    "b2_description_RR_hip_joint",
    "b2_description_RR_thigh_joint",
    "b2_description_RR_calf_joint",
    "R5a_joint1",
    "R5a_joint2",
    "R5a_joint3",
    "R5a_joint4",
    "R5a_joint5",
    "R5a_joint6",
)

GRIPPER_JOINT_NAMES = ("R5a_joint7", "R5a_joint8")


@dataclass
class IsaacPlantState:
    q: np.ndarray
    dq: np.ndarray
    quat_xyzw: np.ndarray
    base_ang_vel: np.ndarray
    tilt_rad: float


def controlled_joint_indices(joint_names: list[str] | tuple[str, ...]) -> list[int]:
    ids = []
    for name in CONTROLLED_JOINT_NAMES:
        try:
            ids.append(joint_names.index(name))
        except ValueError as exc:
            raise RuntimeError(f"Robot is missing controlled joint {name!r}") from exc
    return ids


def gripper_joint_indices(joint_names: list[str] | tuple[str, ...]) -> list[int]:
    return [joint_names.index(name) for name in GRIPPER_JOINT_NAMES if name in joint_names]


def _quat_wxyz_to_xyzw(quat_wxyz) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=np.float32).reshape(4)
    return np.array([q[1], q[2], q[3], q[0]], dtype=np.float32)


def _projected_gravity_from_xyzw(quat_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quat_xyzw, dtype=np.float32).reshape(4)
    rot = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    return rot.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)


class IsaacDeployPlantAdapter:
    """Read/write adapter exposing the same plant contract as the real-mirror FSM."""

    def __init__(self, robot):
        self.robot = robot
        self.joint_ids = controlled_joint_indices(list(robot.joint_names))
        self.gripper_ids = gripper_joint_indices(list(robot.joint_names))
        self._joint_ids_tensor = torch.tensor(self.joint_ids, dtype=torch.long, device=robot.device)
        self._gripper_ids_tensor = torch.tensor(self.gripper_ids, dtype=torch.long, device=robot.device)

    @property
    def device(self):
        return self.robot.device

    def read_state(self) -> IsaacPlantState:
        q = self.robot.data.joint_pos[0, self._joint_ids_tensor].detach().cpu().numpy().astype(np.float32)
        dq = self.robot.data.joint_vel[0, self._joint_ids_tensor].detach().cpu().numpy().astype(np.float32)
        quat_xyzw = _quat_wxyz_to_xyzw(self.robot.data.root_quat_w[0].detach().cpu().numpy())
        base_ang_vel = self.robot.data.root_ang_vel_b[0].detach().cpu().numpy().astype(np.float32)
        projected_gravity = _projected_gravity_from_xyzw(quat_xyzw)
        tilt = math.acos(max(-1.0, min(1.0, -float(projected_gravity[2]))))
        return IsaacPlantState(q=q, dq=dq, quat_xyzw=quat_xyzw, base_ang_vel=base_ang_vel, tilt_rad=tilt)

    def apply_targets(self, controlled_q_target: np.ndarray, gripper_q_target: torch.Tensor | None = None) -> None:
        target = torch.as_tensor(controlled_q_target, dtype=torch.float32, device=self.device).reshape(1, -1)
        self.robot.set_joint_position_target(target, joint_ids=self._joint_ids_tensor)
        if self.gripper_ids and gripper_q_target is not None:
            self.robot.set_joint_position_target(gripper_q_target, joint_ids=self._gripper_ids_tensor)

    def write_control_gains(self, stiffness, damping) -> None:
        stiffness_t = torch.as_tensor(stiffness, dtype=torch.float32, device=self.device).reshape(1, -1)
        damping_t = torch.as_tensor(damping, dtype=torch.float32, device=self.device).reshape(1, -1)
        self.robot.write_joint_stiffness_to_sim(stiffness_t, joint_ids=self._joint_ids_tensor)
        self.robot.write_joint_damping_to_sim(damping_t, joint_ids=self._joint_ids_tensor)


class B2ArxIsaacPolicyController:
    """Deployment FSM host for the IsaacLab scene."""

    def __init__(
        self,
        robot,
        *,
        deploy_yaml: str | Path = DEFAULT_POLICY_DEPLOY_YAML,
        onnx_path: str | Path = DEFAULT_POLICY_ONNX,
        start_state: str = "Passive",
        command_source: CommandSource | None = None,
        ee_sphere=None,
        auto_arm_loco: bool = False,
        arm_ema_tau: float = 0.02,
        arm_sync_tau: float = 0.17,
    ):
        self.robot = robot
        self.plant = IsaacDeployPlantAdapter(robot)
        self.runtime = DeployPolicyRuntime(deploy_yaml, onnx_path=onnx_path)
        self.command_source = command_source or ScriptedCommandSource()
        self.command_buffer = ArmLocoCommandBuffer()
        if ee_sphere is not None:
            self.command_buffer.set(ee_sphere)
            # ArmLocoState.enter() calls buf.reset_to_init() on every ArmLoco entry,
            # which would overwrite the configured sphere with the hardcoded INIT.
            # Pin INIT to the configured sphere so reset restores it, not the default.
            self.command_buffer.INIT = tuple(self.command_buffer.get())
        self.auto_arm_loco = bool(auto_arm_loco)
        self.arm_sync_tau = float(arm_sync_tau)
        self._control_acc = 0.0
        self._state_elapsed = 0.0
        self._last_state_name = start_state
        self._last_q_target = self.runtime.offset.copy()
        self._last_kp = self.runtime.stiffness.copy()
        self._last_kd = self.runtime.damping.copy()
        self._last_raw_action = np.zeros(self.runtime.action_dim, dtype=np.float32)

        passive = PassiveState()
        fixstand = FixStandState()
        fixstand.bind_plant(self.plant)
        prealign = ArmPreAlignState(arm_target=self.runtime.offset[12:18], leg_hold_target=FixStandState.QS2)
        prealign.bind_plant(self.plant)
        loco = ArmLocoState(
            self.runtime,
            self.command_buffer,
            control_dt=self.runtime.step_dt,
            arm_ema_tau=arm_ema_tau,
        )
        self.fsm = CtrlFSM(
            {
                "Passive": passive,
                "FixStand": fixstand,
                "ArmPreAlign": prealign,
                "ArmLoco": loco,
            },
            initial=start_state,
            plant=self.plant,
        )

    @property
    def state_name(self) -> str:
        return self.fsm.current_name

    @property
    def last_raw_action(self) -> np.ndarray:
        current = self.fsm.states.get("ArmLoco")
        if isinstance(current, ArmLocoState):
            return current.last_raw_action.copy()
        return self._last_raw_action.copy()

    @property
    def last_q_target(self) -> np.ndarray:
        return self._last_q_target.copy()

    @property
    def control_dt(self) -> float:
        return self.runtime.step_dt

    def reset(self) -> None:
        self._control_acc = 0.0
        self._state_elapsed = 0.0
        self._last_state_name = self.fsm.current_name
        self._last_q_target = self.runtime.offset.copy()
        self._last_kp = self.runtime.stiffness.copy()
        self._last_kd = self.runtime.damping.copy()
        self._apply_gains_for_state()
        self.plant.apply_targets(self._last_q_target, self._default_gripper_target())

    def close(self) -> None:
        """Release the command source (carb subscriptions)."""
        self.command_source.close()

    def update(self, sim_dt: float) -> None:
        self._control_acc += float(sim_dt)
        if self._control_acc + 1.0e-9 >= self.runtime.step_dt:
            self._control_acc -= self.runtime.step_dt
            self._tick_control()
        self.plant.apply_targets(self._last_q_target, self._default_gripper_target())

    def _tick_control(self) -> None:
        state = self.plant.read_state()
        command = self._command_for_current_state()
        if self.fsm.current_name == "ArmPreAlign":
            prealign = self.fsm.states["ArmPreAlign"]
            assert isinstance(prealign, ArmPreAlignState)
            prealign.update_ready(arm_q=state.q[12:18], dt=self.runtime.step_dt)
        if self.fsm.current_name == "FixStand" and command.arm_prealign_pressed:
            fixstand = self.fsm.states["FixStand"]
            prealign = self.fsm.states["ArmPreAlign"]
            assert isinstance(fixstand, FixStandState)
            assert isinstance(prealign, ArmPreAlignState)
            prealign.set_leg_hold_target(fixstand.run(self.plant, command, t=self._state_elapsed)[:12])

        q_target = self.fsm.tick(self.plant, command, tilt_rad=state.tilt_rad, stale=self.command_source.is_stale(), t=self._state_elapsed)
        if self.fsm.current_name != self._last_state_name:
            self._state_elapsed = 0.0
            self._last_state_name = self.fsm.current_name
            q_target = self.fsm.current.run(self.plant, command, t=0.0)
        else:
            self._state_elapsed += self.runtime.step_dt

        self._last_q_target = np.asarray(q_target, dtype=np.float32).reshape(18)
        self._apply_gains_for_state()

    def _command_for_current_state(self) -> ArmLocoCommand:
        cmd = self.command_source.poll()
        if self.auto_arm_loco:
            if self.fsm.current_name == "Passive":
                cmd.fixstand_pressed = True
            elif self.fsm.current_name == "FixStand" and self._state_elapsed >= FixStandState.TS[-1]:
                cmd.arm_prealign_pressed = True
            elif self.fsm.current_name == "ArmPreAlign":
                prealign = self.fsm.states["ArmPreAlign"]
                if isinstance(prealign, ArmPreAlignState) and prealign.ready:
                    cmd.arm_loco_pressed = True
        return cmd

    def _apply_gains_for_state(self) -> None:
        if self.fsm.current_name == "ArmLoco":
            kp = self.runtime.stiffness.copy()
            kd = self.runtime.damping.copy()
        elif self.fsm.current_name == "Passive":
            kp = np.zeros(18, dtype=np.float32)
            kd = np.full(18, PassiveState.kd, dtype=np.float32)
        else:
            kp = self.runtime.stiffness.copy()
            kd = self.runtime.damping.copy()
            kp[:12] = FixStandState.kp
            kd[:12] = FixStandState.kd
            # Mirror real-mirror non-policy arm external controller by making the Isaac arm a soft position hold.
            kp[12:18] = self.runtime.stiffness[12:18]
            kd[12:18] = self.runtime.damping[12:18]
        if not np.allclose(kp, self._last_kp) or not np.allclose(kd, self._last_kd):
            self.plant.write_control_gains(kp, kd)
            self._last_kp = kp
            self._last_kd = kd

    def _default_gripper_target(self) -> torch.Tensor | None:
        if not self.plant.gripper_ids:
            return None
        default = self.robot.data.default_joint_pos[:, self.plant._gripper_ids_tensor]
        return default
