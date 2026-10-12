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
    projected_gravity_from_xyzw,
)
from .runtime import DeployPolicyRuntime
from .command_sources import CommandSource, ScriptedCommandSource


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_POLICY_RUN_DIR = REPOSITORY_ROOT / "models" / "basic_locomotion"
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
    projected_gravity: np.ndarray | None = None


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


class IsaacDeployPlantAdapter:
    """Read/write adapter exposing the same plant contract as the real-mirror FSM."""

    def __init__(self, robot):
        self.robot = robot
        self.joint_ids = controlled_joint_indices(list(robot.joint_names))
        self.gripper_ids = gripper_joint_indices(list(robot.joint_names))
        self._joint_ids_tensor = torch.tensor(self.joint_ids, dtype=torch.long, device=robot.device)
        self._gripper_ids_tensor = torch.tensor(self.gripper_ids, dtype=torch.long, device=robot.device)
        self._cached_state: IsaacPlantState | None = None

    @property
    def device(self):
        return self.robot.device

    def invalidate_state(self) -> None:
        """Drop the cached state; call whenever the simulation has advanced."""
        self._cached_state = None

    def read_state(self) -> IsaacPlantState:
        # Physics does not advance within a control tick, so every FSM state
        # shares one read (and one device->host transfer) per tick.
        if self._cached_state is None:
            self._cached_state = self._read_state()
        return self._cached_state

    def _read_state(self) -> IsaacPlantState:
        data = self.robot.data
        # One device->host sync for all state the FSM and policy need.
        packed = torch.cat(
            (
                data.joint_pos[0, self._joint_ids_tensor],
                data.joint_vel[0, self._joint_ids_tensor],
                data.root_quat_w[0],
                data.root_ang_vel_b[0],
            )
        ).detach().to(device="cpu", dtype=torch.float32).numpy()
        n = len(self.joint_ids)
        q = packed[:n].copy()
        dq = packed[n : 2 * n].copy()
        quat_xyzw = _quat_wxyz_to_xyzw(packed[2 * n : 2 * n + 4])
        base_ang_vel = packed[2 * n + 4 : 2 * n + 7].copy()
        projected_gravity = projected_gravity_from_xyzw(quat_xyzw)
        tilt = math.acos(max(-1.0, min(1.0, -float(projected_gravity[2]))))
        return IsaacPlantState(
            q=q,
            dq=dq,
            quat_xyzw=quat_xyzw,
            base_ang_vel=base_ang_vel,
            tilt_rad=tilt,
            projected_gravity=projected_gravity,
        )

    def apply_targets(self, controlled_q_target: np.ndarray, gripper_q_target: torch.Tensor | None = None) -> None:
        target = torch.as_tensor(controlled_q_target, dtype=torch.float32, device=self.device).reshape(1, -1)
        self.robot.set_joint_position_target(target, joint_ids=self._joint_ids_tensor)
        if gripper_q_target is not None:
            self.robot.set_joint_position_target(gripper_q_target, joint_ids=self._gripper_ids_tensor)

    def default_gripper_target(self) -> torch.Tensor | None:
        if not self.gripper_ids:
            return None
        return self.robot.data.default_joint_pos[:, self._gripper_ids_tensor].clone()

    def write_control_gains(self, stiffness, damping, joint_ids) -> None:
        stiffness_t = torch.as_tensor(stiffness, dtype=torch.float32, device=self.device).reshape(1, -1)
        damping_t = torch.as_tensor(damping, dtype=torch.float32, device=self.device).reshape(1, -1)
        joint_ids_t = torch.as_tensor(joint_ids, dtype=torch.long, device=self.device)
        self.robot.write_joint_stiffness_to_sim(stiffness_t, joint_ids=joint_ids_t)
        self.robot.write_joint_damping_to_sim(damping_t, joint_ids=joint_ids_t)


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
        self._last_state_name = start_state
        self._gripper_target = self.plant.default_gripper_target()
        self._reset_tracking()

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
        self._loco = loco
        self._prealign = prealign
        self._fixstand = fixstand

    @property
    def state_name(self) -> str:
        return self.fsm.current_name

    @property
    def last_raw_action(self) -> np.ndarray:
        return self._loco.last_raw_action.copy()

    @property
    def last_q_target(self) -> np.ndarray:
        return self._last_q_target.copy()

    @property
    def last_command(self) -> ArmLocoCommand:
        return ArmLocoCommand(**vars(self._last_command))

    @property
    def control_dt(self) -> float:
        return self.runtime.step_dt

    def _reset_tracking(self) -> None:
        self._control_acc = 0.0
        self._state_elapsed = 0.0
        self._last_q_target = self.runtime.offset.copy()
        self._last_kp = np.full_like(self.runtime.stiffness, np.nan, dtype=np.float32)
        self._last_kd = np.full_like(self.runtime.damping, np.nan, dtype=np.float32)
        self._last_command = ArmLocoCommand()

    def reset(self) -> None:
        self.command_source.reset()
        self._last_state_name = self.fsm.current_name
        self._reset_tracking()
        self.plant.invalidate_state()
        # The gripper is not policy-controlled; its default target is constant.
        self._gripper_target = self.plant.default_gripper_target()
        self._apply_gains_for_state()
        self.plant.apply_targets(self._last_q_target, self._gripper_target)

    def close(self) -> None:
        """Release the command source (carb subscriptions)."""
        self.command_source.close()

    def update(self, sim_dt: float) -> None:
        sim_dt = float(sim_dt)
        self._control_acc += sim_dt
        if self._control_acc + 1.0e-9 >= self.runtime.step_dt:
            self._control_acc -= self.runtime.step_dt
            self._tick_control()
        if self.fsm.current_name == "ArmLoco":
            self._last_q_target[ArmLocoState.ARM_SLICE] = self._loco.advance_arm_ema(sim_dt)
        self.plant.apply_targets(self._last_q_target, self._gripper_target)

    def _tick_control(self) -> None:
        plant = self.plant
        plant.invalidate_state()
        state = plant.read_state()
        command = self._command_for_current_state()
        if self.fsm.current_name == "ArmPreAlign":
            self._prealign.update_ready(arm_q=state.q[12:18], dt=self.runtime.step_dt)
        if self.fsm.current_name == "FixStand" and command.arm_prealign_pressed:
            self._prealign.set_leg_hold_target(self._fixstand.run(plant, command, t=self._state_elapsed)[:12])

        # Transition first, then run the (possibly new) state exactly once, so a
        # newly entered ArmLoco pushes one observation and infers once.
        if self.fsm.transition(plant, command, tilt_rad=state.tilt_rad, stale=self.command_source.is_stale()):
            self._state_elapsed = 0.0
        q_target = self.fsm.current.run(plant, command, t=self._state_elapsed)
        if self.fsm.current_name == self._last_state_name:
            self._state_elapsed += self.runtime.step_dt
        self._last_state_name = self.fsm.current_name

        self._last_q_target = np.asarray(q_target, dtype=np.float32).reshape(18)
        # Cheap when unchanged; re-asserts gains if anything reset the actuators.
        self._apply_gains_for_state()

    def _command_for_current_state(self) -> ArmLocoCommand:
        cmd = self.command_source.poll()
        if self.auto_arm_loco:
            if self.fsm.current_name == "Passive":
                cmd.fixstand_pressed = True
            elif self.fsm.current_name == "FixStand" and self._state_elapsed >= FixStandState.TS[-1]:
                cmd.arm_prealign_pressed = True
            elif self.fsm.current_name == "ArmPreAlign" and self.fsm.states["ArmPreAlign"].ready:
                cmd.arm_loco_pressed = True
        self._last_command = ArmLocoCommand(**vars(cmd))
        return cmd

    def _apply_gains_for_state(self) -> None:
        if self.fsm.current_name == "Passive":
            kp = np.zeros(18, dtype=np.float32)
            kd = np.full(18, PassiveState.kd, dtype=np.float32)
        else:
            kp = self.runtime.stiffness.copy()
            kd = self.runtime.damping.copy()
            if self.fsm.current_name != "ArmLoco":
                # Legs use the stand gains; the arm keeps its policy gains as a soft
                # position hold, mirroring the real robot's external arm controller.
                kp[:12] = FixStandState.kp
                kd[:12] = FixStandState.kd
        if not np.allclose(kp, self._last_kp) or not np.allclose(kd, self._last_kd):
            self._write_actuator_gains(kp, kd)
            self._last_kp = kp.copy()
            self._last_kd = kd.copy()

    def _write_actuator_gains(self, stiffness, damping) -> None:
        """Update actuator gains using IsaacLab's explicit/implicit split.

        IdealPDActuator computes torques from actuator.stiffness/damping, while
        implicit actuators need their PhysX drive gains updated. Keep explicit
        PhysX drives untouched so the sim remains a pure explicit-PD mirror.
        """
        stiffness_t = torch.as_tensor(stiffness, dtype=torch.float32, device=self.plant.device).reshape(-1)
        damping_t = torch.as_tensor(damping, dtype=torch.float32, device=self.plant.device).reshape(-1)
        controlled_index_by_joint = {int(joint_id): i for i, joint_id in enumerate(self.plant.joint_ids)}
        for actuator in self.robot.actuators.values():
            local_ids, controlled_ids, sim_joint_ids = self._actuator_controlled_indices(
                actuator.joint_indices, controlled_index_by_joint
            )
            if not sim_joint_ids:
                continue
            local_ids_t = torch.as_tensor(local_ids, dtype=torch.long, device=self.plant.device)
            selected_stiffness = stiffness_t[torch.as_tensor(controlled_ids, dtype=torch.long, device=self.plant.device)]
            selected_damping = damping_t[torch.as_tensor(controlled_ids, dtype=torch.long, device=self.plant.device)]
            actuator_envs = actuator.stiffness.shape[0]
            stiffness_values = selected_stiffness.reshape(1, -1).expand(actuator_envs, -1)
            damping_values = selected_damping.reshape(1, -1).expand(actuator_envs, -1)
            actuator.stiffness[:, local_ids_t] = stiffness_values
            actuator.damping[:, local_ids_t] = damping_values
            if getattr(actuator, "is_implicit_model", False):
                self.plant.write_control_gains(stiffness_values, damping_values, joint_ids=sim_joint_ids)

    def _actuator_controlled_indices(self, joint_indices, controlled_index_by_joint) -> tuple[list[int], list[int], list[int]]:
        if isinstance(joint_indices, slice):
            sim_joint_ids = list(range(*joint_indices.indices(self.robot.num_joints)))
        elif isinstance(joint_indices, torch.Tensor):
            sim_joint_ids = [int(i) for i in joint_indices.detach().cpu().reshape(-1).tolist()]
        else:
            sim_joint_ids = [int(i) for i in np.asarray(joint_indices).reshape(-1).tolist()]

        local_ids = []
        controlled_ids = []
        controlled_sim_joint_ids = []
        for local_id, sim_joint_id in enumerate(sim_joint_ids):
            controlled_id = controlled_index_by_joint.get(sim_joint_id)
            if controlled_id is None:
                continue
            local_ids.append(local_id)
            controlled_ids.append(controlled_id)
            controlled_sim_joint_ids.append(sim_joint_id)
        return local_ids, controlled_ids, controlled_sim_joint_ids

