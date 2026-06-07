from __future__ import annotations


class ButtonEdgeFilter:
    """Rising-edge detector for analog gamepad buttons.

    carb gamepad callbacks fire on both press and release with event.value in
    [0, 1]. A rising edge is prev < thresh and current >= thresh. Used by
    GamepadCommandSource; keyboard KEY_PRESS callbacks are already clean edges.
    """

    def __init__(self, press_thresh: float = 0.5) -> None:
        self.press_thresh = float(press_thresh)
        self._prev: dict[str, float] = {}

    def update(self, name: str, value: float) -> bool:
        prev = self._prev.get(name, 0.0)
        cur = float(value)
        self._prev[name] = cur
        return prev < self.press_thresh <= cur

    def reset(self) -> None:
        self._prev = {}
