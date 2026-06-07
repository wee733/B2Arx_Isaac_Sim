from __future__ import annotations

from scripts.policy_deploy.command_sources.latch import _CommandLatch
from scripts.policy_deploy.command_sources.edge import ButtonEdgeFilter
from scripts.policy_deploy.command_sources.base import ScriptedCommandSource


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
