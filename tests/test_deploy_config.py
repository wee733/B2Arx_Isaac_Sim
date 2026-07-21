from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from scripts.policy_deploy.deploy_config import (
    DeployConfig,
    InputSettings,
    PolicyConfig,
    file_sha256,
    load_deploy_config,
    verify_policy_bundle,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_CHECKPOINT_SHA256 = "dab9197db9ecec7c1496d23e548b02c1a7e22e17badbd075d5270f5cdb9630b4"
BASELINE_ONNX_SHA256 = "10b79e8531fdd1cb455d20c2079fe0c5b6dea6e9797dacf2061584e143c48f60"
BASELINE_DEPLOY_SHA256 = "dac88692cf90dc173a25ae3144a7ded5ff583075c98666858fbbcf6b8fa652d6"


def _make_test_bundle(tmp_path: Path) -> PolicyConfig:
    source_run = tmp_path / "source_run"
    bundle = tmp_path / "bundle"
    checkpoint = source_run / "model_1.pt"
    onnx = bundle / "exported" / "policy_full.onnx"
    deploy_yaml = bundle / "params" / "deploy.yaml"
    checkpoint.parent.mkdir(parents=True)
    onnx.parent.mkdir(parents=True)
    deploy_yaml.parent.mkdir(parents=True)
    checkpoint.write_bytes(b"checkpoint")
    onnx.write_bytes(b"onnx")
    deploy_yaml.write_text("step_dt: 0.02\n", encoding="utf-8")
    manifest = bundle / "bundle_manifest.txt"
    manifest.write_text(
        "\n".join(
            [
                "model_id=test_model",
                f"checkpoint={checkpoint}",
                f"run_dir={source_run}",
                f"checkpoint_sha256={file_sha256(checkpoint)}",
                f"policy_full_onnx_sha256={file_sha256(onnx)}",
                f"deploy_yaml_sha256={file_sha256(deploy_yaml)}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return PolicyConfig(
        name="test_model",
        run_dir=bundle,
        checkpoint=checkpoint,
        manifest=manifest,
    )


def test_input_settings_defaults() -> None:
    cfg = DeployConfig.from_dict({})
    assert cfg.input.backend == "scripted"
    assert cfg.input.keyboard["v_x_sensitivity"] == 0.8
    assert cfg.input.keyboard["v_y_sensitivity"] == 0.4
    assert cfg.input.keyboard["omega_z_sensitivity"] == 1.0
    assert cfg.input.gamepad["v_x_sensitivity"] == 0.3
    assert cfg.input.gamepad["v_y_sensitivity"] == 0.2
    assert cfg.input.gamepad["omega_z_sensitivity"] == 0.3
    assert cfg.input.gamepad["dead_zone"] == 0.01


def test_input_settings_parsed_from_yaml() -> None:
    data = yaml.safe_load(
        """
        input:
          backend: keyboard
          keyboard:
            v_x_sensitivity: 1.5
          gamepad:
            dead_zone: 0.05
        """
    )
    cfg = DeployConfig.from_dict(data)
    assert cfg.input.backend == "keyboard"
    # explicit override wins; unspecified keys fall back to defaults
    assert cfg.input.keyboard["v_x_sensitivity"] == 1.5
    assert cfg.input.keyboard["v_y_sensitivity"] == 0.4
    assert cfg.input.gamepad["dead_zone"] == 0.05


def test_policy_config_records_checkpoint_and_manifest_metadata() -> None:
    data = yaml.safe_load(
        """
        policy:
          run_dir: /tmp/run
          checkpoint: /tmp/run/model_14000.pt
          manifest: /tmp/run/exported/deploy_manifest.txt
        """
    )
    cfg = DeployConfig.from_dict(data)
    assert str(cfg.policy.checkpoint) == "/tmp/run/model_14000.pt"
    assert str(cfg.policy.manifest) == "/tmp/run/exported/deploy_manifest.txt"
    assert cfg.policy.resolved_onnx().name == "policy_full.onnx"


def test_policy_bundle_verification_accepts_matching_immutable_bundle(tmp_path) -> None:
    cfg = _make_test_bundle(tmp_path)

    values = verify_policy_bundle(cfg)

    assert values["model_id"] == "test_model"


def test_policy_bundle_verification_rejects_asset_hash_mismatch(tmp_path) -> None:
    cfg = _make_test_bundle(tmp_path)
    cfg.resolved_onnx().write_bytes(b"wrong-onnx")

    with pytest.raises(ValueError, match="ONNX SHA256 mismatch"):
        verify_policy_bundle(cfg)


def test_policy_bundle_verification_rejects_checkpoint_path_mismatch(tmp_path) -> None:
    cfg = _make_test_bundle(tmp_path)
    other_checkpoint = cfg.checkpoint.parent / "model_2.pt"
    other_checkpoint.write_bytes(cfg.checkpoint.read_bytes())
    cfg.checkpoint = other_checkpoint

    with pytest.raises(ValueError, match="checkpoint path does not match manifest"):
        verify_policy_bundle(cfg)


def test_policy_bundle_verification_accepts_relative_manifest_paths(tmp_path) -> None:
    bundle = tmp_path / "basic_locomotion"
    checkpoint = bundle / "basic_locomotion_model.pt"
    onnx = bundle / "exported" / "policy_full.onnx"
    deploy_yaml = bundle / "params" / "deploy.yaml"
    onnx.parent.mkdir(parents=True)
    deploy_yaml.parent.mkdir(parents=True)
    checkpoint.write_bytes(b"checkpoint")
    onnx.write_bytes(b"onnx")
    deploy_yaml.write_text("step_dt: 0.02\n", encoding="utf-8")
    manifest = bundle / "bundle_manifest.txt"
    manifest.write_text(
        "\n".join(
            [
                "model_id=basic_locomotion",
                "checkpoint=basic_locomotion_model.pt",
                "run_dir=.",
                f"checkpoint_sha256={file_sha256(checkpoint)}",
                f"policy_full_onnx_sha256={file_sha256(onnx)}",
                f"deploy_yaml_sha256={file_sha256(deploy_yaml)}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    cfg = PolicyConfig(
        name="basic_locomotion",
        run_dir=bundle,
        checkpoint=checkpoint,
        manifest=manifest,
    )

    values = verify_policy_bundle(cfg)

    assert values["model_id"] == "basic_locomotion"


def test_load_deploy_config_resolves_relative_paths_against_profile(tmp_path) -> None:
    profile_dir = tmp_path / "config" / "policies"
    profile_dir.mkdir(parents=True)
    profile = profile_dir / "demo.yaml"
    profile.write_text(
        "\n".join(
            [
                "policy:",
                "  run_dir: ../../models/demo",
                "  checkpoint: ../../models/demo/model.pt",
                "  manifest: ../../models/demo/bundle_manifest.txt",
                "scene:",
                "  robot_usd: ../../assets/robot.usd",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    cfg = load_deploy_config(profile)

    assert cfg.policy.run_dir == (tmp_path / "models" / "demo").resolve()
    assert cfg.policy.checkpoint == (tmp_path / "models" / "demo" / "model.pt").resolve()
    assert cfg.scene.robot_usd == (tmp_path / "assets" / "robot.usd").resolve()


def test_default_nav2_profile_is_the_portable_basic_locomotion_bundle() -> None:
    cfg = load_deploy_config(REPO_ROOT / "config" / "policies" / "basic_locomotion.yaml")

    values = verify_policy_bundle(cfg.policy)

    assert cfg.policy.name == "basic_locomotion"
    assert cfg.policy.checkpoint.name == "basic_locomotion_model.pt"
    assert cfg.policy.run_dir == REPO_ROOT / "models" / "basic_locomotion"
    assert values["checkpoint_sha256"] == BASELINE_CHECKPOINT_SHA256
    assert values["policy_full_onnx_sha256"] == BASELINE_ONNX_SHA256
    assert values["deploy_yaml_sha256"] == BASELINE_DEPLOY_SHA256
    assert cfg.deploy.arm_ema_tau == pytest.approx(0.02)
    assert cfg.scene.robot_usd is None


def test_deploy_settings_no_longer_has_input_backend() -> None:
    cfg = DeployConfig.from_dict({"deploy": {"start_state": "ArmLoco"}})
    assert not hasattr(cfg.deploy, "input_backend")
    assert cfg.deploy.start_state == "ArmLoco"


def test_deploy_settings_parse_arm_ema_tau() -> None:
    cfg = DeployConfig.from_dict({"deploy": {"arm_ema_tau": 0.035}})
    assert cfg.deploy.arm_ema_tau == pytest.approx(0.035)
