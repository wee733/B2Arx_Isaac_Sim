from __future__ import annotations

import math
from collections.abc import Sequence


ZERO_TWIST = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class CommandHold:
    """Hold a Twist briefly, then produce zero until a new command arrives."""

    def __init__(self, timeout_s: float) -> None:
        self.timeout_s = float(timeout_s)
        if self.timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be positive, got {self.timeout_s}")
        self._command = ZERO_TWIST
        self._last_rx_s: float | None = None

    def update(self, command: Sequence[float], now_s: float) -> None:
        values = tuple(float(value) for value in command)
        if len(values) != len(ZERO_TWIST):
            raise ValueError(f"Twist command must have 6 components, got {len(values)}")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Twist command contains a non-finite value")
        self._command = values
        self._last_rx_s = float(now_s)

    def sample(self, now_s: float) -> tuple[float, float, float, float, float, float]:
        if self._last_rx_s is None:
            return ZERO_TWIST
        if float(now_s) - self._last_rx_s >= self.timeout_s:
            return ZERO_TWIST
        return self._command

    def invalidate(self) -> None:
        """Fail safe immediately after an invalid upstream command."""
        self._command = ZERO_TWIST
        self._last_rx_s = None
