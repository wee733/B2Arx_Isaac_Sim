from __future__ import annotations

import yaml

from scripts.policy_deploy.deploy_config import DeployConfig, InputSettings


def test_input_settings_defaults() -> None:
    cfg = DeployConfig.from_dict({})
    assert cfg.input.backend == "scripted"
    assert cfg.input.keyboard["v_x_sensitivity"] == 0.8
    assert cfg.input.keyboard["v_y_sensitivity"] == 0.4
    assert cfg.input.keyboard["omega_z_sensitivity"] == 1.0
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


def test_deploy_settings_no_longer_has_input_backend() -> None:
    cfg = DeployConfig.from_dict({"deploy": {"start_state": "ArmLoco"}})
    assert not hasattr(cfg.deploy, "input_backend")
    assert cfg.deploy.start_state == "ArmLoco"
