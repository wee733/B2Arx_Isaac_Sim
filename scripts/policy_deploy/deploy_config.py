from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml


_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")


def _resolve_path(value, *, base_dir: Path | None = None):
    """Resolve a local path relative to its YAML file when one is supplied."""
    if value is None or value == "":
        return None
    text = str(value)
    if text.startswith(("http://", "https://", "omniverse://")):
        return text
    path = Path(text).expanduser()
    if base_dir is not None and not path.is_absolute():
        path = base_dir / path
    return path.resolve()


@dataclass
class PolicyConfig:
    """One deployable policy: a training run_dir plus optional explicit overrides.

    onnx and deploy_yaml default to the standard rsl_rl export layout under run_dir:
    ``run_dir/exported/policy_full.onnx`` and ``run_dir/params/deploy.yaml``.
    checkpoint and manifest trace which .pt produced the exported ONNX bundle.
    When a manifest is configured, startup verifies the checkpoint, ONNX, and
    deploy.yaml paths and SHA256 values before inference is allowed to start.
    """

    name: str = "default"
    run_dir: Path | None = None
    onnx: Path | None = None
    deploy_yaml: Path | None = None
    checkpoint: Path | None = None
    manifest: Path | None = None

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
    def from_dict(
        cls,
        data: dict,
        *,
        name: str = "default",
        base_dir: Path | None = None,
    ) -> "PolicyConfig":
        return cls(
            name=str(data.get("name", name)),
            run_dir=_resolve_path(data.get("run_dir"), base_dir=base_dir),
            onnx=_resolve_path(data.get("onnx"), base_dir=base_dir),
            deploy_yaml=_resolve_path(data.get("deploy_yaml"), base_dir=base_dir),
            checkpoint=_resolve_path(data.get("checkpoint"), base_dir=base_dir),
            manifest=_resolve_path(data.get("manifest"), base_dir=base_dir),
        )


def file_sha256(path: str | Path) -> str:
    """Return a lowercase SHA256 digest without loading the whole asset at once."""
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_policy_manifest(path: str | Path) -> dict[str, str]:
    """Parse the line-oriented key=value manifests emitted by the export tools."""
    manifest_path = Path(path).expanduser().resolve()
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Policy manifest not found: {manifest_path}")

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(manifest_path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"Invalid policy manifest line {line_number}: {raw_line!r}")
        key, value = (part.strip() for part in line.split("=", 1))
        if not key or not value:
            raise ValueError(f"Invalid policy manifest line {line_number}: {raw_line!r}")
        if key in values:
            raise ValueError(f"Duplicate policy manifest key {key!r} in {manifest_path}")
        values[key] = value
    return values


def _manifest_path(values: dict[str, str], key: str, manifest_path: Path) -> Path:
    value = values.get(key)
    if value is None:
        raise ValueError(f"Policy manifest {manifest_path} is missing required key {key!r}")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = manifest_path.parent / path
    return path.resolve()


def _manifest_sha256(values: dict[str, str], key: str, manifest_path: Path) -> str:
    value = values.get(key, "").lower()
    if not _SHA256_RE.fullmatch(value):
        raise ValueError(f"Policy manifest {manifest_path} has invalid {key}: {value!r}")
    return value


def verify_policy_bundle(policy: PolicyConfig) -> dict[str, str]:
    """Fail closed if a configured policy bundle is not the manifested export.

    Both export layouts used by the B2ARX tooling are supported:

    - a source run manifest at ``run_dir/exported/deploy_manifest.txt``;
    - an immutable bundle manifest at ``bundle_dir/bundle_manifest.txt`` whose
      ``run_dir`` still names the source training run.
    """
    if policy.manifest is None:
        raise ValueError(f"policy {policy.name!r}: manifest is required for bundle verification")
    if policy.checkpoint is None:
        raise ValueError(f"policy {policy.name!r}: checkpoint is required when manifest is configured")

    manifest_path = Path(policy.manifest).expanduser().resolve()
    values = parse_policy_manifest(manifest_path)
    checkpoint = Path(policy.checkpoint).expanduser().resolve()
    onnx = Path(policy.resolved_onnx()).expanduser().resolve()
    deploy_yaml = Path(policy.resolved_deploy_yaml()).expanduser().resolve()
    for label, path in (("checkpoint", checkpoint), ("ONNX", onnx), ("deploy.yaml", deploy_yaml)):
        if not path.is_file():
            raise FileNotFoundError(f"Policy {label} not found: {path}")

    manifested_checkpoint = _manifest_path(values, "checkpoint", manifest_path)
    manifested_run_dir = _manifest_path(values, "run_dir", manifest_path)
    if checkpoint != manifested_checkpoint:
        raise ValueError(
            f"Policy checkpoint path does not match manifest: configured={checkpoint}, "
            f"manifest={manifested_checkpoint}"
        )
    if checkpoint.parent != manifested_run_dir:
        raise ValueError(
            f"Policy manifest run_dir does not own checkpoint: run_dir={manifested_run_dir}, "
            f"checkpoint={checkpoint}"
        )

    if policy.run_dir is not None:
        configured_run_dir = Path(policy.run_dir).expanduser().resolve()
        if "model_id" in values:
            # Immutable bundles keep source run_dir metadata but place the
            # actual ONNX/deploy pair next to bundle_manifest.txt.
            expected_bundle_dir = manifest_path.parent
            if configured_run_dir != expected_bundle_dir:
                raise ValueError(
                    f"Policy bundle run_dir does not contain its manifest: "
                    f"run_dir={configured_run_dir}, manifest_dir={expected_bundle_dir}"
                )
        elif configured_run_dir != manifested_run_dir:
            raise ValueError(
                f"Policy run_dir does not match source manifest: configured={configured_run_dir}, "
                f"manifest={manifested_run_dir}"
            )

    expected_hashes = {
        "checkpoint": _manifest_sha256(values, "checkpoint_sha256", manifest_path),
        "ONNX": _manifest_sha256(values, "policy_full_onnx_sha256", manifest_path),
        "deploy.yaml": _manifest_sha256(values, "deploy_yaml_sha256", manifest_path),
    }
    actual_paths = {"checkpoint": checkpoint, "ONNX": onnx, "deploy.yaml": deploy_yaml}
    for label, path in actual_paths.items():
        actual = file_sha256(path)
        expected = expected_hashes[label]
        if actual != expected:
            raise ValueError(
                f"Policy {label} SHA256 mismatch for {path}: expected={expected}, actual={actual}"
            )

    source_manifest_value = values.get("source_manifest")
    if source_manifest_value:
        source_manifest = _manifest_path(values, "source_manifest", manifest_path)
        source_values = parse_policy_manifest(source_manifest)
        mirrored_keys = (
            "checkpoint",
            "run_dir",
            "checkpoint_sha256",
            "policy_full_onnx_sha256",
            "deploy_yaml_sha256",
        )
        for key in mirrored_keys:
            if source_values.get(key) != values.get(key):
                raise ValueError(
                    f"Policy bundle manifest key {key!r} does not match source manifest "
                    f"{source_manifest}"
                )

    return values


@dataclass
class SceneConfig:
    environment_usd: object = None  # Path | str(URL) | None
    robot_usd: Path | None = None

    @classmethod
    def from_dict(cls, data: dict, *, base_dir: Path | None = None) -> "SceneConfig":
        return cls(
            environment_usd=_resolve_path(data.get("environment_usd"), base_dir=base_dir),
            robot_usd=_resolve_path(data.get("robot_usd"), base_dir=base_dir),
        )


@dataclass
class DeploySettings:
    start_state: str = "Passive"
    auto_arm_loco: bool = False
    ee_sphere: list[float] | None = None
    command: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    arm_ema_tau: float = 0.02

    @classmethod
    def from_dict(cls, data: dict) -> "DeploySettings":
        ee = data.get("ee_sphere")
        cmd = data.get("command", [0.0, 0.0, 0.0])
        return cls(
            start_state=str(data.get("start_state", "Passive")),
            auto_arm_loco=bool(data.get("auto_arm_loco", False)),
            ee_sphere=[float(v) for v in ee] if ee is not None else None,
            command=[float(v) for v in cmd],
            arm_ema_tau=float(data.get("arm_ema_tau", 0.02)),
        )


_KEYBOARD_DEFAULTS = {"v_x_sensitivity": 0.8, "v_y_sensitivity": 0.4, "omega_z_sensitivity": 1.0}
_GAMEPAD_DEFAULTS = {
    "v_x_sensitivity": 0.3,
    "v_y_sensitivity": 0.2,
    "omega_z_sensitivity": 0.3,
    "dead_zone": 0.01,
}
_ROS2_TWIST_DEFAULTS = {
    "topic": "/cmd_vel",
    "heartbeat_topic": "/cmd_vel_heartbeat",
    "timeout_s": 0.5,
    "passive_timeout_s": 5.0,
    "max_vx": 0.8,
    "max_vy": 0.5,
    "max_wz": 0.6,
    # model_29999 was trained on either a full stop or planar walking at
    # >=0.25 m/s.  It never saw a pure (vx=0, vy=0, wz!=0) command.
    "min_planar_speed": 0.25,
    # At the non-zero floor, the exported |wz| <= 0.6 rad/s contract gives
    # Rmin = 0.25 / 0.6 = 0.4167 m. Keep a small numerical margin.
    "min_turn_radius": 0.42,
    "zero_epsilon": 0.01,
}


@dataclass
class InputSettings:
    """Live-input backend selection plus per-device sensitivities.

    Field names align with IsaacLab Se2KeyboardCfg / Se2GamepadCfg so values pass
    straight through to the device cfg.
    """

    backend: str = "scripted"  # scripted / keyboard / gamepad / ros2_twist
    keyboard: dict = field(default_factory=lambda: dict(_KEYBOARD_DEFAULTS))
    gamepad: dict = field(default_factory=lambda: dict(_GAMEPAD_DEFAULTS))
    ros2_twist: dict = field(default_factory=lambda: dict(_ROS2_TWIST_DEFAULTS))

    @classmethod
    def from_dict(cls, data: dict) -> "InputSettings":
        data = data or {}
        kbd = dict(_KEYBOARD_DEFAULTS)
        kbd.update(data.get("keyboard") or {})
        pad = dict(_GAMEPAD_DEFAULTS)
        pad.update(data.get("gamepad") or {})
        twist = dict(_ROS2_TWIST_DEFAULTS)
        twist.update(data.get("ros2_twist") or {})
        return cls(
            backend=str(data.get("backend", "scripted")),
            keyboard=kbd,
            gamepad=pad,
            ros2_twist=twist,
        )


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
    def from_dict(cls, data: dict, *, base_dir: Path | None = None) -> "DeployConfig":
        data = data or {}
        policies_raw = data.get("policies") or []
        policies = [
            PolicyConfig.from_dict(p, name=p.get("name", f"policy_{i}"), base_dir=base_dir)
            for i, p in enumerate(policies_raw)
        ]
        # Active single policy: explicit `policy:` block, else first of `policies:`.
        if "policy" in data and data["policy"]:
            policy = PolicyConfig.from_dict(data["policy"], base_dir=base_dir)
        elif policies:
            policy = policies[0]
        else:
            policy = PolicyConfig()
        return cls(
            policy=policy,
            scene=SceneConfig.from_dict(data.get("scene") or {}, base_dir=base_dir),
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
    return DeployConfig.from_dict(data, base_dir=cfg_path.parent)
