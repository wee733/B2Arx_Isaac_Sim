from __future__ import annotations

import pytest

from scripts.policy_deploy.command_sources.latch import _CommandLatch
from scripts.policy_deploy.command_sources.edge import ButtonEdgeFilter
from scripts.policy_deploy.command_sources.base import ScriptedCommandSource
from scripts.policy_deploy.command_sources import make_command_source
from scripts.policy_deploy.command_sources.gamepad_mapping import GamepadMirrorMapper
from scripts.policy_deploy.deploy_config import DeployConfig


def test_latch_set_then_poll_reads_and_clears() -> None:
    latch = _CommandLatch()
    latch.set("fixstand_pressed")
    latch.set("ee_step", 1)
    out = latch.poll()
    assert out["fixstand_pressed"] is True
    assert out["ee_step"] == 1
    # second poll is empty again
    assert latch.poll() == {}


def test_latch_merges_same_event_within_tick() -> None:
    latch = _CommandLatch()
    latch.set("fixstand_pressed")
    latch.set("fixstand_pressed")
    out = latch.poll()
    assert out["fixstand_pressed"] is True  # merged, not counted
    assert latch.poll() == {}


def test_latch_ee_step_last_write_wins() -> None:
    latch = _CommandLatch()
    latch.set("ee_step", 1)
    latch.set("ee_step", -1)
    assert latch.poll()["ee_step"] == -1


def test_edge_filter_rising_only() -> None:
    f = ButtonEdgeFilter(press_thresh=0.5)
    assert f.update("A", 0.0) is False
    assert f.update("A", 1.0) is True    # 0 -> 1 rising
    assert f.update("A", 1.0) is False   # held, no repeat
    assert f.update("A", 0.0) is False   # release
    assert f.update("A", 0.9) is True    # rising again


def test_edge_filter_below_threshold_is_release() -> None:
    f = ButtonEdgeFilter(press_thresh=0.5)
    assert f.update("B", 0.4) is False   # below thresh = not pressed
    assert f.update("B", 0.6) is True    # crosses thresh
    assert f.update("B", 0.45) is False  # drops below = release
    assert f.update("B", 0.7) is True    # rising again


def test_gamepad_mapper_edges_match_mirror_buttons() -> None:
    mapper = GamepadMirrorMapper()

    assert mapper.update("A", 1.0) == {"fixstand_pressed": True}
    assert mapper.update("A", 1.0) == {}
    assert mapper.update("A", 0.0) == {}
    assert mapper.update("LEFT_STICK", 1.0) == {"arm_prealign_pressed": True}
    assert mapper.update("Y", 1.0) == {"arm_loco_pressed": True}
    assert mapper.update("MENU", 1.0) == {"arm_loco_pressed": True}
    assert mapper.update("B", 1.0) == {"passive_pressed": True}
    assert mapper.update("X", 1.0) == {"ee_cycle_dim": True}
    assert mapper.update("BACK", 1.0) == {"ee_reset": True}
    assert mapper.update("RIGHT_STICK", 1.0) == {"ee_reset": True}


def test_gamepad_mapper_accepts_carb_official_input_names() -> None:
    mapper = GamepadMirrorMapper(max_vx=0.3, max_vy=0.2, max_wz=0.3)

    assert mapper.update("ButtonA", 1.0) == {"fixstand_pressed": True}
    assert mapper.update("Menu2", 1.0) == {"arm_loco_pressed": True}
    assert mapper.update("Menu1", 1.0) == {"ee_reset": True}
    mapper.update("DpadUp", 1.0)
    mapper.update("LeftShoulder", 1.0)
    assert mapper.resolve_velocity((0.0, 0.0, 0.0)) == pytest.approx((0.3, 0.0, 0.3))


def test_gamepad_mapper_dpad_matches_mirror_velocity() -> None:
    mapper = GamepadMirrorMapper(max_vx=0.3, max_vy=0.2, max_wz=0.3)

    assert mapper.update("DPAD_UP", 1.0) == {}
    assert mapper.resolve_velocity((0.0, 0.0, 0.0)) == pytest.approx((0.3, 0.0, 0.0))
    mapper.update("DPAD_UP", 0.0)
    mapper.update("DPAD_DOWN", 1.0)
    assert mapper.resolve_velocity((0.0, 0.0, 0.0)) == pytest.approx((-0.3, 0.0, 0.0))
    mapper.update("DPAD_DOWN", 0.0)
    mapper.update("DPAD_RIGHT", 1.0)
    assert mapper.resolve_velocity((0.0, 0.0, 0.0)) == pytest.approx((0.0, -0.2, 0.0))
    mapper.update("DPAD_RIGHT", 0.0)
    mapper.update("DPAD_LEFT", 1.0)
    assert mapper.resolve_velocity((0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.2, 0.0))


def test_gamepad_mapper_bumpers_match_mirror_yaw() -> None:
    mapper = GamepadMirrorMapper(max_wz=0.3)

    mapper.update("LEFT_BUMPER", 1.0)
    assert mapper.resolve_velocity((0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, 0.3))
    mapper.update("LEFT_BUMPER", 0.0)
    mapper.update("RIGHT_BUMPER", 1.0)
    assert mapper.resolve_velocity((0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, -0.3))


def test_gamepad_mapper_triggers_are_held_ee_steps() -> None:
    mapper = GamepadMirrorMapper()

    assert mapper.update("RIGHT_TRIGGER", 1.0) == {}
    assert mapper.held_command_fields() == {
        "ee_step_positive_held": True,
        "ee_step_negative_held": False,
    }
    mapper.update("LEFT_TRIGGER", 1.0)
    assert mapper.held_command_fields() == {
        "ee_step_positive_held": True,
        "ee_step_negative_held": True,
    }
    mapper.update("RIGHT_TRIGGER", 0.0)
    assert mapper.held_command_fields() == {
        "ee_step_positive_held": False,
        "ee_step_negative_held": True,
    }


def test_gamepad_mapper_flips_isaac_stick_signs_to_mirror_semantics() -> None:
    mapper = GamepadMirrorMapper()

    # IsaacLab Se2Gamepad reports +vy for left-stick-right and +wz for right-stick-right;
    # the MuJoCo mirror consumes those as negative vy/wz.
    assert mapper.resolve_velocity((0.3, 0.2, 0.3)) == pytest.approx((0.3, -0.2, -0.3))


def test_scripted_source_returns_configured_velocity() -> None:
    src = ScriptedCommandSource(command=[0.3, -0.1, 0.2])
    cmd = src.poll()
    assert (cmd.vx, cmd.vy, cmd.wz) == (0.3, -0.1, 0.2)
    # no auto_arm_loco leakage: state-transition flags stay False
    assert cmd.fixstand_pressed is False
    assert cmd.arm_loco_pressed is False


def test_scripted_source_poll_returns_fresh_object_each_tick() -> None:
    src = ScriptedCommandSource(command=[0.0, 0.0, 0.0])
    a = src.poll()
    a.fixstand_pressed = True          # caller mutates (controller overlays auto flags)
    b = src.poll()
    assert b.fixstand_pressed is False  # next tick is unaffected
    assert a is not b


def test_scripted_source_is_stale_false_and_close_noop() -> None:
    src = ScriptedCommandSource(command=[0.0, 0.0, 0.0])
    assert src.is_stale() is False
    src.close()  # must not raise


def test_factory_scripted_does_not_import_carb() -> None:
    import sys
    cfg = DeployConfig.from_dict({"deploy": {"command": [0.1, 0.2, 0.3]},
                                  "input": {"backend": "scripted"}})
    src = make_command_source(cfg.input, cfg.deploy)
    assert isinstance(src, ScriptedCommandSource)
    assert src.poll().vx == 0.1
    assert "carb" not in sys.modules  # scripted path must stay carb-free


def test_factory_rejects_unknown_backend() -> None:
    cfg = DeployConfig.from_dict({"input": {"backend": "joystick3000"}})
    with pytest.raises(ValueError, match="joystick3000"):
        make_command_source(cfg.input, cfg.deploy)
