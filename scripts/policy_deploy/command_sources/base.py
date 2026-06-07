from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence

from ..fsm import ArmLocoCommand


class CommandSource(ABC):
    """Turns one input device into one ArmLocoCommand per control tick.

    The controller consumes only this interface, never the device. Future
    backends (ROS2 topic, Isaac ROS vision, remote Thor command) implement the
    same contract.
    """

    @abstractmethod
    def poll(self) -> ArmLocoCommand:
        """Return a FRESH ArmLocoCommand for this tick (caller may mutate it)."""

    def reset(self) -> None:
        """Reset internal state (history, latches). Default: no-op."""

    def is_stale(self, now_s: float | None = None) -> bool:
        """True when the source has lost contact (watchdog). Default: never.

        Local devices (keyboard/gamepad/scripted) never go stale. Network
        sources override this to drive the FSM back to Passive via fsm.tick(stale=).
        """
        del now_s
        return False

    def close(self) -> None:
        """Release device resources (carb subscriptions). Default: no-op."""


class ScriptedCommandSource(CommandSource):
    """Emits a fixed (vx, vy, wz). Used for headless smoke and no-input runs.

    Does NOT set auto_arm_loco flags — that logic lives in the controller.
    """

    def __init__(self, command: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        vx, vy, wz = (float(v) for v in command)
        self._vx, self._vy, self._wz = vx, vy, wz

    def poll(self) -> ArmLocoCommand:
        return ArmLocoCommand(vx=self._vx, vy=self._vy, wz=self._wz)
