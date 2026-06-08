from __future__ import annotations

import yaml

from scripts.policy_deploy.deploy_config import DeployConfig, InputSettings


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


def test_deploy_settings_no_longer_has_input_backend() -> None:
    cfg = DeployConfig.from_dict({"deploy": {"start_state": "ArmLoco"}})
    assert not hasattr(cfg.deploy, "input_backend")
    assert cfg.deploy.start_state == "ArmLoco"
