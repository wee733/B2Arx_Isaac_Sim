from __future__ import annotations

import pytest

from ros_ws.src.b2arx_nav2_bringup.b2arx_nav2_bringup.cmd_vel_watchdog_core import (
    CommandHold,
    ZERO_TWIST,
)


def test_command_hold_starts_and_times_out_to_zero() -> None:
    hold = CommandHold(timeout_s=0.5)
    assert hold.sample(10.0) == ZERO_TWIST

    command = (0.4, -0.1, 0.0, 0.0, 0.0, 0.3)
    hold.update(command, now_s=10.0)
    assert hold.sample(10.4999) == command
    assert hold.sample(10.5) == ZERO_TWIST


def test_command_hold_accepts_identical_zero_heartbeats() -> None:
    hold = CommandHold(timeout_s=0.5)
    hold.update(ZERO_TWIST, now_s=10.0)
    assert hold.sample(10.4) == ZERO_TWIST
    hold.update(ZERO_TWIST, now_s=10.4)
    assert hold.sample(10.8) == ZERO_TWIST


@pytest.mark.parametrize(
    "command",
    [
        (0.0,) * 5,
        (0.0, 0.0, 0.0, 0.0, 0.0, float("nan")),
        (0.0, 0.0, 0.0, 0.0, 0.0, float("inf")),
    ],
)
def test_command_hold_rejects_invalid_twists(command) -> None:
    hold = CommandHold(timeout_s=0.5)
    with pytest.raises(ValueError):
        hold.update(command, now_s=0.0)


def test_command_hold_rejects_nonpositive_timeout() -> None:
    with pytest.raises(ValueError, match="positive"):
        CommandHold(timeout_s=0.0)


def test_command_hold_invalidate_fails_safe_immediately() -> None:
    hold = CommandHold(timeout_s=0.5)
    hold.update((0.4, 0.0, 0.0, 0.0, 0.0, 0.1), now_s=10.0)
    hold.invalidate()
    assert hold.sample(10.1) == ZERO_TWIST
