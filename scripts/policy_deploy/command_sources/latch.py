from __future__ import annotations

from typing import Any


class _CommandLatch:
    """Stores one-shot edge events from the carb callback thread until the next
    control-tick poll(). poll() reads accumulated events and clears them.

    Same-named events within one tick are merged (last write wins); no counting.
    v1 guarantees "at least once", not per-event delivery.
    """

    def __init__(self) -> None:
        self._pending: dict[str, Any] = {}

    def set(self, name: str, value: Any = True) -> None:
        self._pending[name] = value

    def poll(self) -> dict[str, Any]:
        out = self._pending
        self._pending = {}
        return out

    def reset(self) -> None:
        self._pending = {}
