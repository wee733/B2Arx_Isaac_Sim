from __future__ import annotations

import carb
import carb.input
from isaaclab.devices.keyboard.se2_keyboard import Se2Keyboard, Se2KeyboardCfg
from isaaclab.devices.gamepad.se2_gamepad import Se2Gamepad, Se2GamepadCfg

from ..fsm import ArmLocoCommand
from .base import CommandSource
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


class KeyboardCommandSource(CommandSource):
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


# Gamepad button (carb enum name) -> latch event. One-shot, edge-filtered.
_GAMEPAD_BUTTON_EVENTS = {
    "A": ("fixstand_pressed", True),
    "LEFT_STICK": ("arm_prealign_pressed", True),
    "Y": ("arm_loco_pressed", True),
    "MENU": ("arm_loco_pressed", True),
    "B": ("passive_pressed", True),
    "X": ("ee_cycle_dim", True),
    "RIGHT_STICK": ("ee_reset", True),
    "RIGHT_TRIGGER": ("ee_step", 1),
    "LEFT_TRIGGER": ("ee_step", -1),
}


class GamepadCommandSource(CommandSource):
    """Wraps Se2Gamepad for velocity; self-subscribes carb for edge-filtered buttons.

    Se2Gamepad.add_callback() fires func() with no args, so it cannot read
    event.value and cannot distinguish press from release. Buttons are therefore
    handled by a second carb subscription this class owns and releases in close().
    """

    def __init__(self, sensitivity: dict) -> None:
        import omni

        cfg = Se2GamepadCfg(
            v_x_sensitivity=float(sensitivity["v_x_sensitivity"]),
            v_y_sensitivity=float(sensitivity["v_y_sensitivity"]),
            omega_z_sensitivity=float(sensitivity["omega_z_sensitivity"]),
            dead_zone=float(sensitivity["dead_zone"]),
        )
        self._device = Se2Gamepad(cfg)
        self._latch = _CommandLatch()
        self._edges = ButtonEdgeFilter(press_thresh=0.5)
        self._input = carb.input.acquire_input_interface()
        self._gamepad = omni.appwindow.get_default_app_window().get_gamepad(0)
        self._sub = self._input.subscribe_to_gamepad_events(
            self._gamepad, self._on_gamepad_event
        )
        print(self._device, flush=True)

    @classmethod
    def try_create(cls, sensitivity: dict) -> "GamepadCommandSource | None":
        """Return a source, or None if no gamepad is attached (caller falls back)."""
        import omni

        gamepad = omni.appwindow.get_default_app_window().get_gamepad(0)
        if gamepad is None:
            return None
        return cls(sensitivity)

    def _on_gamepad_event(self, event, *args) -> bool:
        name = event.input.name  # e.g. "A", "RIGHT_TRIGGER"
        if name in _GAMEPAD_BUTTON_EVENTS and self._edges.update(name, event.value):
            field_name, value = _GAMEPAD_BUTTON_EVENTS[name]
            self._latch.set(field_name, value)
        return True

    def poll(self) -> ArmLocoCommand:
        vx, vy, wz = (float(v) for v in self._device.advance().tolist())
        cmd = ArmLocoCommand(vx=vx, vy=vy, wz=wz)
        for name, value in self._latch.poll().items():
            setattr(cmd, name, value)
        return cmd

    def reset(self) -> None:
        self._device.reset()
        self._latch.reset()
        self._edges.reset()

    def is_stale(self, now_s: float | None = None) -> bool:
        del now_s
        return False

    def close(self) -> None:
        if getattr(self, "_sub", None) is not None:
            self._input.unsubscribe_to_gamepad_events(self._gamepad, self._sub)
            self._sub = None
        self._device = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
