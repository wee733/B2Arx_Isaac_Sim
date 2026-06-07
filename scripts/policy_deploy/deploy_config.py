from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml


def _resolve_path(value):
    """Resolve a local path; leave Isaac/Nucleus URLs and empties untouched."""
    if value is None or value == "":
        return None
    text = str(value)
    if text.startswith(("http://", "https://", "omniverse://")):
        return text
    return Path(text).expanduser().resolve()


@dataclass
class PolicyConfig:
    """One deployable policy: a training run_dir plus optional explicit overrides.

    onnx and deploy_yaml default to the standard rsl_rl export layout under run_dir:
    ``run_dir/exported/policy_full.onnx`` and ``run_dir/params/deploy.yaml``.
    """

    name: str = "default"
    run_dir: Path | None = None
    onnx: Path | None = None
    deploy_yaml: Path | None = None

    def resolved_onnx(self) -> Path:
        if self.onnx is not None:
            return self.onnx
        if self.run_dir is None:
            raise ValueError(f"policy {self.name!r}: need run_dir or explicit onnx")
        return self.run_dir / "exported" / "policy_full.onnx"

    def resolved_deploy_yaml(self) -> Path:
        if self.deploy_yaml is not None:
            return self.deploy_yaml
        if self.run_dir is None:
            raise ValueError(f"policy {self.name!r}: need run_dir or explicit deploy_yaml")
        return self.run_dir / "params" / "deploy.yaml"

    @classmethod
    def from_dict(cls, data: dict, *, name: str = "default") -> "PolicyConfig":
        return cls(
            name=str(data.get("name", name)),
            run_dir=_resolve_path(data.get("run_dir")),
            onnx=_resolve_path(data.get("onnx")),
            deploy_yaml=_resolve_path(data.get("deploy_yaml")),
        )


@dataclass
class SceneConfig:
    environment_usd: object = None  # Path | str(URL) | None
    robot_usd: Path | None = None

    @classmethod
    def from_dict(cls, data: dict) -> "SceneConfig":
        return cls(
            environment_usd=_resolve_path(data.get("environment_usd")),
            robot_usd=_resolve_path(data.get("robot_usd")),
        )


@dataclass
class DeploySettings:
    start_state: str = "Passive"
    arm_gain_profile: str = "identified"
    auto_arm_loco: bool = False
    ee_sphere: list[float] | None = None
    command: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    @classmethod
    def from_dict(cls, data: dict) -> "DeploySettings":
        ee = data.get("ee_sphere")
        cmd = data.get("command", [0.0, 0.0, 0.0])
        return cls(
            start_state=str(data.get("start_state", "Passive")),
            arm_gain_profile=str(data.get("arm_gain_profile", "identified")),
            auto_arm_loco=bool(data.get("auto_arm_loco", False)),
            ee_sphere=[float(v) for v in ee] if ee is not None else None,
            command=[float(v) for v in cmd],
        )


_KEYBOARD_DEFAULTS = {"v_x_sensitivity": 0.8, "v_y_sensitivity": 0.4, "omega_z_sensitivity": 1.0}
_GAMEPAD_DEFAULTS = {
    "v_x_sensitivity": 1.0,
    "v_y_sensitivity": 1.0,
    "omega_z_sensitivity": 1.0,
    "dead_zone": 0.01,
}


@dataclass
class InputSettings:
    """Live-input backend selection plus per-device sensitivities.

    Field names align with IsaacLab Se2KeyboardCfg / Se2GamepadCfg so values pass
    straight through to the device cfg.
    """

    backend: str = "scripted"  # scripted / keyboard / gamepad
    keyboard: dict = field(default_factory=lambda: dict(_KEYBOARD_DEFAULTS))
    gamepad: dict = field(default_factory=lambda: dict(_GAMEPAD_DEFAULTS))

    @classmethod
    def from_dict(cls, data: dict) -> "InputSettings":
        data = data or {}
        kbd = dict(_KEYBOARD_DEFAULTS)
        kbd.update(data.get("keyboard") or {})
        pad = dict(_GAMEPAD_DEFAULTS)
        pad.update(data.get("gamepad") or {})
        return cls(backend=str(data.get("backend", "scripted")), keyboard=kbd, gamepad=pad)


@dataclass
class DeployConfig:
    """Top-level deployment configuration: policy + scene + deploy settings.

    Loaded from a hand-written deploy_config.yaml. The active policy is a single
    PolicyConfig (method A). The ``policies`` list is reserved for future multi-policy
    hot-switching (method C): when populated, ``policy`` is the initially active one.
    """

    policy: PolicyConfig = field(default_factory=PolicyConfig)
    scene: SceneConfig = field(default_factory=SceneConfig)
    deploy: DeploySettings = field(default_factory=DeploySettings)
    input: InputSettings = field(default_factory=InputSettings)
    # Reserved for method C. Empty today; each entry is a selectable PolicyConfig.
    policies: list[PolicyConfig] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict) -> "DeployConfig":
        data = data or {}
        policies_raw = data.get("policies") or []
        policies = [
            PolicyConfig.from_dict(p, name=p.get("name", f"policy_{i}"))
            for i, p in enumerate(policies_raw)
        ]
        # Active single policy: explicit `policy:` block, else first of `policies:`.
        if "policy" in data and data["policy"]:
            policy = PolicyConfig.from_dict(data["policy"])
        elif policies:
            policy = policies[0]
        else:
            policy = PolicyConfig()
        return cls(
            policy=policy,
            scene=SceneConfig.from_dict(data.get("scene") or {}),
            deploy=DeploySettings.from_dict(data.get("deploy") or {}),
            input=InputSettings.from_dict(data.get("input") or {}),
            policies=policies,
        )


def load_deploy_config(path) -> DeployConfig:
    """Load a deploy_config.yaml into a DeployConfig."""
    cfg_path = Path(path).expanduser().resolve()
    if not cfg_path.is_file():
        raise FileNotFoundError(f"deploy_config not found: {cfg_path}")
    with cfg_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"deploy_config must be a YAML mapping, got {type(data).__name__}")
    return DeployConfig.from_dict(data)
