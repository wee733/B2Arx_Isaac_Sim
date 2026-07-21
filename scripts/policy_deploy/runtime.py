from __future__ import annotations

import math
from collections import deque
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


OBS_TERM_ORDER = [
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

LOCKED_JOINT6_ACTION_INDEX = 17
WALK_DEADBAND = 0.1
# The training command sampler and both established deployment paths floor any
# intentional planar command to 0.25 m/s.  The policy has not seen walking
# commands in (0, 0.25), so Isaac must preserve the same input contract.
MIN_WALK_SPEED = 0.25


def sanitize_velocity_command(
    vx: float,
    vy: float,
    wz: float,
    *,
    min_speed: float = MIN_WALK_SPEED,
) -> tuple[float, float, float]:
    """Match the real/MuJoCo minimum planar walking-speed contract."""
    vx = float(vx)
    vy = float(vy)
    wz = float(wz)
    speed = math.hypot(vx, vy)
    if speed == 0.0:
        return 0.0, 0.0, wz
    if speed < float(min_speed):
        scale = float(min_speed) / speed
        vx *= scale
        vy *= scale
    return vx, vy, wz


def gait_phase_train_semantics(
    prev_phase: float,
    vx: float,
    vy: float,
    wz: float,
    control_dt: float,
    period: float,
) -> tuple[list[float], float]:
    """Training semantics: gait phase advances only for planar walking commands."""
    del wz
    walking = math.hypot(float(vx), float(vy)) > WALK_DEADBAND
    if walking:
        phase = (float(prev_phase) + float(control_dt) / float(period)) % 1.0
    else:
        phase = 0.0
    return [math.sin(2.0 * math.pi * phase), math.cos(2.0 * math.pi * phase)], phase


def build_obs_termwise(
    obs_terms: Iterable[tuple[str, Iterable[float]]],
    *,
    proj_grav_xy,
    base_ang_vel,
    joint_pos_rel,
    joint_vel,
    last_action,
    gait_sincos,
    vel_cmd,
    ee_sphere,
) -> np.ndarray:
    """Build one deployment observation frame from deploy.yaml term order and scales."""
    raw = {
        "projected_gravity_xy": np.asarray(proj_grav_xy, dtype=np.float32),
        "base_ang_vel": np.asarray(base_ang_vel, dtype=np.float32),
        "joint_pos_rel": np.asarray(joint_pos_rel, dtype=np.float32),
        "joint_vel_rel": np.asarray(joint_vel, dtype=np.float32),
        "last_action": np.asarray(last_action, dtype=np.float32),
        "gait_phase": np.asarray(gait_sincos, dtype=np.float32),
        "velocity_commands": np.asarray(vel_cmd, dtype=np.float32),
        "ee_goal_sphere": np.asarray(ee_sphere, dtype=np.float32),
        "zero_force_obs": np.zeros(6, dtype=np.float32),
    }
    parts = []
    for name, scale in obs_terms:
        if name not in raw:
            raise KeyError(f"Unsupported observation term: {name}")
        parts.append(raw[name] * np.asarray(scale, dtype=np.float32))
    return np.concatenate(parts).astype(np.float32)


class DeployPolicyRuntime:
    """Nested deploy.yaml + optional policy_full.onnx runtime.

    The semantics mirror b2arx_sim2sim2real's real-mirror runtime:
    observation terms are scaled term-wise, history is maintained as a flat gym-style
    stack, last_action stores raw actions, and q_target is decoded as offset + raw * scale.
    """

    def __init__(self, deploy_yaml_path: str | Path, onnx_path: str | Path | None = None):
        self.deploy_yaml_path = Path(deploy_yaml_path).expanduser().resolve()
        with self.deploy_yaml_path.open("r", encoding="utf-8") as f:
            self.deploy = yaml.safe_load(f)

        obs_h = self.deploy["observations"]["obs_history"]
        missing_obs_terms = [name for name in OBS_TERM_ORDER if name not in obs_h]
        if missing_obs_terms:
            raise ValueError(f"deploy.yaml observations.obs_history is missing terms: {missing_obs_terms}")
        unexpected_obs_terms = set(obs_h) - {"use_gym_history", *OBS_TERM_ORDER}
        if unexpected_obs_terms:
            raise ValueError(
                f"deploy.yaml observations.obs_history has unsupported terms: {sorted(unexpected_obs_terms)}"
            )
        self.obs_terms = [(name, obs_h[name]["scale"]) for name in OBS_TERM_ORDER]
        self.single_obs_dim = int(sum(len(scale) for _, scale in self.obs_terms))
        self.frame_stack = int(obs_h[self.obs_terms[0][0]].get("history_length", 30))

        action_cfg = self.deploy["actions"]["B2ArxJointPositionAction"]
        self.scale = np.asarray(action_cfg["scale"], dtype=np.float32)
        self.offset = np.asarray(action_cfg["offset"], dtype=np.float32)
        self.joint_lo = np.asarray(action_cfg["joint_lo"], dtype=np.float32)
        self.joint_hi = np.asarray(action_cfg["joint_hi"], dtype=np.float32)
        self.step_dt = float(self.deploy.get("step_dt", 0.02))
        self.stiffness = np.asarray(self.deploy.get("stiffness", [0.0] * self.action_dim), dtype=np.float32)
        self.damping = np.asarray(self.deploy.get("damping", [0.0] * self.action_dim), dtype=np.float32)
        self.period = float(obs_h["gait_phase"]["params"]["period"])

        self._validate_shapes()
        self.sess = None
        self.in_name = None
        self.out_name = None
        if onnx_path is not None:
            self.load_onnx(onnx_path)
        self._hist: deque[np.ndarray] = deque(maxlen=self.frame_stack)

    @property
    def action_dim(self) -> int:
        return int(self.scale.size)

    @property
    def history_dim(self) -> int:
        return int(self.frame_stack * self.single_obs_dim)

    def _validate_shapes(self) -> None:
        expected = self.action_dim
        if expected != 18:
            raise ValueError(f"action dimension must be 18, got {expected}")
        for name, arr in (
            ("offset", self.offset),
            ("joint_lo", self.joint_lo),
            ("joint_hi", self.joint_hi),
            ("stiffness", self.stiffness),
            ("damping", self.damping),
        ):
            if arr.shape != (expected,):
                raise ValueError(f"{name} must have shape ({expected},), got {arr.shape}")
        if self.single_obs_dim != 73:
            raise ValueError(f"single observation dimension must be 73, got {self.single_obs_dim}")
        history_lengths = {
            name: int(self.deploy["observations"]["obs_history"][name].get("history_length", 30))
            for name in OBS_TERM_ORDER
        }
        if set(history_lengths.values()) != {self.frame_stack}:
            raise ValueError(f"observation history lengths must match: {history_lengths}")
        if self.frame_stack <= 0:
            raise ValueError(f"history length must be positive, got {self.frame_stack}")
        if self.step_dt <= 0.0:
            raise ValueError(f"step_dt must be positive, got {self.step_dt}")
        if self.period <= 0.0:
            raise ValueError(f"gait period must be positive, got {self.period}")

    def load_onnx(self, onnx_path: str | Path) -> None:
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError(
                "onnxruntime is required for policy inference. Activate the isaaclab conda env or install onnxruntime."
            ) from exc

        path = str(Path(onnx_path).expanduser().resolve())
        self.sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        inputs = self.sess.get_inputs()
        outputs = self.sess.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError(
                f"policy ONNX must have exactly one input and one output, got {len(inputs)} and {len(outputs)}"
            )
        expected_input_shape = [1, self.history_dim]
        expected_output_shape = [1, self.action_dim]
        if list(inputs[0].shape) != expected_input_shape:
            raise ValueError(
                f"policy ONNX input must have shape {expected_input_shape}, got {inputs[0].shape}"
            )
        if list(outputs[0].shape) != expected_output_shape:
            raise ValueError(
                f"policy ONNX output must have shape {expected_output_shape}, got {outputs[0].shape}"
            )
        if inputs[0].type != "tensor(float)" or outputs[0].type != "tensor(float)":
            raise ValueError(
                f"policy ONNX input/output must be tensor(float), got {inputs[0].type} and {outputs[0].type}"
            )
        self.in_name = inputs[0].name
        self.out_name = outputs[0].name

    def reset_history(self, current_obs) -> None:
        obs = self._validate_obs(current_obs)
        self._hist.clear()
        for _ in range(self.frame_stack):
            self._hist.append(obs.copy())

    def push(self, obs) -> None:
        self._hist.append(self._validate_obs(obs))

    def _validate_obs(self, obs) -> np.ndarray:
        arr = np.asarray(obs, dtype=np.float32).reshape(-1)
        if arr.shape != (self.single_obs_dim,):
            raise ValueError(f"observation must have shape ({self.single_obs_dim},), got {arr.shape}")
        return arr

    def obs_history_flat(self) -> np.ndarray:
        if len(self._hist) != self.frame_stack:
            raise RuntimeError("policy history is not initialized; call reset_history() first")
        return np.concatenate(list(self._hist)).astype(np.float32)

    def infer(self) -> np.ndarray:
        if self.sess is None or self.in_name is None or self.out_name is None:
            raise RuntimeError("infer requires an ONNX session; pass onnx_path when constructing DeployPolicyRuntime")
        x = self.obs_history_flat().reshape(1, -1)
        out = self.sess.run([self.out_name], {self.in_name: x})[0]
        return out.reshape(-1).astype(np.float32)

    def lock_action(self, raw_action) -> np.ndarray:
        raw = np.asarray(raw_action, dtype=np.float32).reshape(-1).copy()
        if raw.shape != (self.action_dim,):
            raise ValueError(f"raw_action must have shape ({self.action_dim},), got {raw.shape}")
        raw[LOCKED_JOINT6_ACTION_INDEX] = 0.0
        return raw

    def decode(self, raw_action) -> np.ndarray:
        raw = np.asarray(raw_action, dtype=np.float32).reshape(-1)
        if raw.shape != (self.action_dim,):
            raise ValueError(f"raw_action must have shape ({self.action_dim},), got {raw.shape}")
        q_target = self.offset + raw * self.scale
        return np.clip(q_target, self.joint_lo, self.joint_hi).astype(np.float32)
