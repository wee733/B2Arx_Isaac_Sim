from __future__ import annotations

import carb
import carb.input
from isaaclab.devices.keyboard.se2_keyboard import Se2Keyboard, Se2KeyboardCfg
from isaaclab.devices.gamepad.se2_gamepad import Se2Gamepad, Se2GamepadCfg

from ..fsm import ArmLocoCommand
from .edge import ButtonEdgeFilter
from .latch import _CommandLatch

# Keyboard letter -> latch event name (one-shot, clean KEY_PRESS edges).
# Avoids IsaacLab reserved keys: L=reset, Z/X=yaw, arrows/numpad=velocity.
_KEYBOARD_EVENTS = {
    "F": "fixstand_pressed",
    "G": "arm_prealign_pressed",
    "H": "arm_loco_pressed",
    "P": "passive_pressed",
    "R": "ee_cycle_dim",
    "O": "ee_reset",
}


class KeyboardCommandSource:
    """Wraps Se2Keyboard: advance() -> vx/vy/wz; add_callback binds discrete keys."""

    def __init__(self, sensitivity: dict) -> None:
        cfg = Se2KeyboardCfg(
            v_x_sensitivity=float(sensitivity["v_x_sensitivity"]),
            v_y_sensitivity=float(sensitivity["v_y_sensitivity"]),
            omega_z_sensitivity=float(sensitivity["omega_z_sensitivity"]),
        )
        self._device = Se2Keyboard(cfg)
        self._latch = _CommandLatch()
        for key, event in _KEYBOARD_EVENTS.items():
            self._device.add_callback(key, lambda e=event: self._latch.set(e))
        self._device.add_callback("I", lambda: self._latch.set("ee_step", 1))
        self._device.add_callback("K", lambda: self._latch.set("ee_step", -1))
        print(self._device, flush=True)

    def poll(self) -> ArmLocoCommand:
        vx, vy, wz = (float(v) for v in self._device.advance().tolist())
        cmd = ArmLocoCommand(vx=vx, vy=vy, wz=wz)
        for name, value in self._latch.poll().items():
            setattr(cmd, name, value)
        return cmd

    def reset(self) -> None:
        self._device.reset()
        self._latch.reset()

    def is_stale(self, now_s: float | None = None) -> bool:
        del now_s
        return False

    def close(self) -> None:
        # Se2Keyboard unsubscribes its keyboard sub in __del__.
        self._device = None
