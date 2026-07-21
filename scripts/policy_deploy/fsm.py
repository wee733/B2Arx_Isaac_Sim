from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .runtime import (
    DeployPolicyRuntime,
    build_obs_termwise,
    gait_phase_train_semantics,
    sanitize_velocity_command,
)


@dataclass
class ArmLocoCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    fixstand_pressed: bool = False
    arm_prealign_pressed: bool = False
    arm_loco_pressed: bool = False
    passive_pressed: bool = False
    ee_cycle_dim: bool = False
    ee_step: int = 0
    ee_step_positive_held: bool = False
    ee_step_negative_held: bool = False
    ee_reset: bool = False


class ArmLocoCommandBuffer:
    INIT = (0.36, 0.56, 0.0)
    STEP_R = 0.02
    STEP_ANGLE = 0.05
    R_LIMITS = (0.30, 0.60)
    PITCH_LIMITS = (-0.5, 1.0)
    YAW_LIMITS = (-1.5, 1.5)

    def __init__(self):
        self.reset_to_init()

    def get(self) -> list[float]:
        return [self._r, self._pitch, self._yaw]

    def set(self, sphere) -> None:
        r, pitch, yaw = np.asarray(sphere, dtype=np.float32).reshape(3)
        self._r = self._clamp(float(r), self.R_LIMITS)
        self._pitch = self._clamp(float(pitch), self.PITCH_LIMITS)
        self._yaw = self._clamp(float(yaw), self.YAW_LIMITS)

    def cycle_selected_dimension(self) -> None:
        self._sel = (self._sel + 1) % 3

    def step_selected_dimension(self, direction: int) -> None:
        direction = int(direction)
        if direction == 0:
            return
        if self._sel == 0:
            self._r = self._clamp(self._r + direction * self.STEP_R, self.R_LIMITS)
        elif self._sel == 1:
            self._pitch = self._clamp(self._pitch + direction * self.STEP_ANGLE, self.PITCH_LIMITS)
        else:
            self._yaw = self._clamp(self._yaw + direction * self.STEP_ANGLE, self.YAW_LIMITS)

    def reset_to_init(self) -> None:
        self._r, self._pitch, self._yaw = self.INIT
        self._sel = 0

    @staticmethod
    def _clamp(value: float, limits: tuple[float, float]) -> float:
        return max(limits[0], min(limits[1], value))


class BaseState:
    def enter(self, plant=None) -> None:
        del plant

    def run(self, plant, command: ArmLocoCommand, t: float | None = None) -> np.ndarray:
        raise NotImplementedError

    def check_transition(self, command: ArmLocoCommand) -> str | None:
        del command
        return None

    def exit(self) -> None:
        pass


class PassiveState(BaseState):
    kp = 0.0
    kd = 10.0

    def run(self, plant, command: ArmLocoCommand, t: float | None = None) -> np.ndarray:
        del command, t
        return plant.read_state().q.copy()

    def check_transition(self, command: ArmLocoCommand) -> str | None:
        if command.fixstand_pressed:
            return "FixStand"
        return None


class FixStandState(BaseState):
    TS = (0.0, 1.0, 3.0)
    QS1 = np.array([0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65, 0.0, 1.36, -2.65], dtype=np.float32)
    QS2 = np.array([0.15, 0.67, -1.32, -0.15, 0.67, -1.32, 0.15, 0.67, -1.32, -0.15, 0.67, -1.32], dtype=np.float32)
    kp = 400.0
    kd = 8.0

    def __init__(self):
        self._plant = None
        self._leg_q0 = None
        self.arm_hold = None

    def bind_plant(self, plant) -> None:
        self._plant = plant

    def enter(self, plant=None) -> None:
        if plant is not None:
            self._plant = plant
        if self._plant is None:
            raise RuntimeError("FixStandState requires bind_plant() or enter(plant)")
        q0 = self._plant.read_state().q.copy()
        self._leg_q0 = q0[:12].copy()
        self.arm_hold = q0[12:18].copy()

    def _leg_interp(self, t: float) -> np.ndarray:
        if self._leg_q0 is None:
            raise RuntimeError("FixStandState.enter() must be called before run()")
        if t <= self.TS[0]:
            return self._leg_q0
        if t <= self.TS[1]:
            alpha = (t - self.TS[0]) / (self.TS[1] - self.TS[0])
            return ((1.0 - alpha) * self._leg_q0 + alpha * self.QS1).astype(np.float32)
        if t <= self.TS[2]:
            alpha = (t - self.TS[1]) / (self.TS[2] - self.TS[1])
            return ((1.0 - alpha) * self.QS1 + alpha * self.QS2).astype(np.float32)
        return self.QS2.copy()

    def run(self, plant, command: ArmLocoCommand, t: float | None = None) -> np.ndarray:
        del plant, command
        if self.arm_hold is None:
            raise RuntimeError("FixStandState.enter() must be called before run()")
        t = self.TS[2] if t is None else float(t)
        q_target = np.empty(18, dtype=np.float32)
        q_target[:12] = self._leg_interp(t)
        q_target[12:18] = self.arm_hold
        return q_target

    def check_transition(self, command: ArmLocoCommand) -> str | None:
        if command.arm_prealign_pressed:
            return "ArmPreAlign"
        return None


class ArmPreAlignState(BaseState):
    TOL_RAD = 0.05
    STABLE_TIME_S = 0.5
    LEG_HOLD_BLEND_S = 0.1

    def __init__(self, arm_target, leg_hold_target):
        self.arm_target = np.asarray(arm_target, dtype=np.float32).reshape(6)
        self.leg_hold = np.asarray(leg_hold_target, dtype=np.float32).reshape(12)
        self.ready = False
        self._stable_acc = 0.0
        self._plant = None
        self._leg_q0 = None

    def bind_plant(self, plant) -> None:
        self._plant = plant

    def set_leg_hold_target(self, leg_target) -> None:
        self.leg_hold = np.asarray(leg_target, dtype=np.float32).reshape(12).copy()

    def enter(self, plant=None) -> None:
        if plant is not None:
            self._plant = plant
        self.ready = False
        self._stable_acc = 0.0
        self._leg_q0 = self._plant.read_state().q[:12].copy() if self._plant is not None else self.leg_hold.copy()

    def update_ready(self, arm_q, dt: float) -> None:
        max_err = float(np.max(np.abs(np.asarray(arm_q, dtype=np.float32).reshape(6) - self.arm_target)))
        if max_err < self.TOL_RAD:
            self._stable_acc += float(dt)
        else:
            self._stable_acc = 0.0
        if self._stable_acc >= self.STABLE_TIME_S:
            self.ready = True

    def run(self, plant, command: ArmLocoCommand, t: float | None = None) -> np.ndarray:
        del plant, command
        q_target = np.empty(18, dtype=np.float32)
        if t is None or t >= self.LEG_HOLD_BLEND_S or self._leg_q0 is None:
            q_target[:12] = self.leg_hold
        else:
            alpha = float(t) / self.LEG_HOLD_BLEND_S
            q_target[:12] = (1.0 - alpha) * self._leg_q0 + alpha * self.leg_hold
        q_target[12:18] = self.arm_target
        return q_target

    def check_transition(self, command: ArmLocoCommand) -> str | None:
        if command.arm_loco_pressed and self.ready:
            return "ArmLoco"
        return None


class ArmLocoState(BaseState):
    ARM_SLICE = slice(12, 18)

    def __init__(
        self,
        policy_runtime: DeployPolicyRuntime,
        command_buffer: ArmLocoCommandBuffer,
        *,
        control_dt: float = 0.02,
        arm_ema_tau: float = 0.02,
    ):
        self.rt = policy_runtime
        self.buf = command_buffer
        self.control_dt = float(control_dt)
        self.arm_ema_tau = float(arm_ema_tau)
        self.last_raw_action = np.zeros(self.rt.action_dim, dtype=np.float32)
        self.arm_smooth: np.ndarray | None = None
        self.arm_decoded_target: np.ndarray | None = None
        self._phase = 0.0
        self._ee_repeat_acc = 0.0

    def enter(self, plant=None) -> None:
        if plant is None:
            raise RuntimeError("ArmLocoState.enter() requires plant")
        self.buf.reset_to_init()
        state = plant.read_state()
        current_obs = self._build_obs(
            state,
            raw_action=np.zeros(self.rt.action_dim, dtype=np.float32),
            phase_sincos=[0.0, 1.0],
            vx=0.0,
            vy=0.0,
            wz=0.0,
        )
        self.rt.reset_history(current_obs)
        self.last_raw_action = np.zeros(self.rt.action_dim, dtype=np.float32)
        self._phase = 0.0
        self._ee_repeat_acc = 0.0
        self.arm_smooth = self.rt.offset[self.ARM_SLICE].copy()
        self.arm_decoded_target = self.arm_smooth.copy()

    def advance_arm_ema(self, dt: float) -> np.ndarray:
        """Advance the arm filter at the physics rate used during training."""
        if self.arm_smooth is None or self.arm_decoded_target is None:
            raise RuntimeError("ArmLocoState.enter() must be called before advancing arm EMA")
        alpha = 1.0 if self.arm_ema_tau <= 0.0 else 1.0 - math.exp(-float(dt) / self.arm_ema_tau)
        self.arm_smooth = self.arm_smooth + alpha * (self.arm_decoded_target - self.arm_smooth)
        return self.arm_smooth.copy()

    def _build_obs(self, state, raw_action, phase_sincos, vx: float, vy: float, wz: float) -> np.ndarray:
        projected_gravity = _projected_gravity_from_xyzw(np.asarray(state.quat_xyzw, dtype=np.float32))
        return build_obs_termwise(
            self.rt.obs_terms,
            proj_grav_xy=projected_gravity[:2],
            base_ang_vel=state.base_ang_vel,
            joint_pos_rel=state.q - self.rt.offset,
            joint_vel=state.dq,
            last_action=raw_action,
            gait_sincos=phase_sincos,
            vel_cmd=np.array([vx, vy, wz], dtype=np.float32),
            ee_sphere=self.buf.get(),
        )

    def run(self, plant, command: ArmLocoCommand, t: float | None = None) -> np.ndarray:
        del t
        if self.arm_smooth is None:
            raise RuntimeError("ArmLocoState.enter() must be called before run()")
        if command.ee_cycle_dim:
            self.buf.cycle_selected_dimension()
        if command.ee_step:
            self.buf.step_selected_dimension(command.ee_step)
        held_step = int(command.ee_step_positive_held) - int(command.ee_step_negative_held)
        if held_step:
            self._ee_repeat_acc += self.control_dt
            if self._ee_repeat_acc >= 0.12:
                self.buf.step_selected_dimension(held_step)
                self._ee_repeat_acc = 0.0
        else:
            self._ee_repeat_acc = 0.12
        if command.ee_reset:
            self.buf.reset_to_init()

        state = plant.read_state()
        vx, vy, wz = sanitize_velocity_command(command.vx, command.vy, command.wz)
        sincos, self._phase = gait_phase_train_semantics(
            self._phase,
            vx,
            vy,
            wz,
            self.control_dt,
            self.rt.period,
        )
        obs = self._build_obs(state, self.last_raw_action, sincos, vx, vy, wz)
        self.rt.push(obs)
        raw = self.rt.lock_action(self.rt.infer())
        self.last_raw_action = raw
        q_target = self.rt.decode(raw)
        # Training decodes a new target at 50 Hz, then applies the arm EMA on
        # each 200 Hz physics step. Store the new target here; the Isaac host
        # advances the filter with the actual sim_dt before every physics step.
        self.arm_decoded_target = q_target[self.ARM_SLICE].copy()
        q_target[self.ARM_SLICE] = self.arm_smooth
        return q_target


class CtrlFSM:
    TILT_LIMIT_RAD = 1.0

    def __init__(self, states: dict[str, BaseState], initial: str, plant=None):
        self.states = states
        self.current_name = initial
        self.states[initial].enter(plant)

    @property
    def current(self) -> BaseState:
        return self.states[self.current_name]

    def _shared_check(self, command: ArmLocoCommand, tilt_rad: float | None, stale: bool) -> str | None:
        if command.passive_pressed:
            return "Passive"
        if tilt_rad is not None and tilt_rad > self.TILT_LIMIT_RAD:
            return "Passive"
        if stale:
            return "Passive"
        return None

    def _transition_to(self, name: str, plant=None) -> None:
        if name == self.current_name:
            return
        self.current.exit()
        self.current_name = name
        self.current.enter(plant)

    def tick(
        self,
        plant,
        command: ArmLocoCommand,
        *,
        tilt_rad: float | None = None,
        stale: bool = False,
        t: float | None = None,
    ) -> np.ndarray:
        next_state = self._shared_check(command, tilt_rad, stale)
        if next_state is not None:
            self._transition_to(next_state, plant)
            return self.current.run(plant, command, t=t)
        next_state = self.current.check_transition(command)
        if next_state is not None:
            self._transition_to(next_state, plant)
        return self.current.run(plant, command, t=t)


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
    gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    return rot.T @ gravity_world
