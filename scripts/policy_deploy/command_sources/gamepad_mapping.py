from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field
import re

from .edge import ButtonEdgeFilter


_EDGE_ALIASES = {
    "A": "fixstand_pressed",
    "LEFT_STICK": "arm_prealign_pressed",
    "LEFT_THUMB": "arm_prealign_pressed",
    "Y": "arm_loco_pressed",
    "MENU": "arm_loco_pressed",
    "MENU2": "arm_loco_pressed",
    "START": "arm_loco_pressed",
    "SPECIALRIGHT": "arm_loco_pressed",
    "B": "passive_pressed",
    "X": "ee_cycle_dim",
    "BACK": "ee_reset",
    "MENU1": "ee_reset",
    "VIEW": "ee_reset",
    "SELECT": "ee_reset",
    "SPECIALLEFT": "ee_reset",
    "RIGHT_STICK": "ee_reset",
    "RIGHT_THUMB": "ee_reset",
}

_HELD_ALIASES = {
    "DPAD_UP": "dpad_up",
    "DPAD_DOWN": "dpad_down",
    "DPAD_RIGHT": "dpad_right",
    "DPAD_LEFT": "dpad_left",
    "D_PAD_UP": "dpad_up",
    "D_PAD_DOWN": "dpad_down",
    "D_PAD_RIGHT": "dpad_right",
    "D_PAD_LEFT": "dpad_left",
    "LEFT_BUMPER": "left_bumper",
    "LEFT_SHOULDER": "left_bumper",
    "LB": "left_bumper",
    "RIGHT_BUMPER": "right_bumper",
    "RIGHT_SHOULDER": "right_bumper",
    "RB": "right_bumper",
    "LEFT_TRIGGER": "left_trigger",
    "LT": "left_trigger",
    "RIGHT_TRIGGER": "right_trigger",
    "RT": "right_trigger",
}


def _norm(input_name: str) -> str:
    raw = str(input_name).upper()
    compact = re.sub(r"[^A-Z0-9]+", "", raw)
    if compact.startswith("BUTTON") and len(compact) == 7:
        return compact[-1]
    aliases = {
        "LEFTSTICK": "LEFT_STICK",
        "LEFTTHUMB": "LEFT_THUMB",
        "LEFTSTICKBUTTON": "LEFT_STICK",
        "RIGHTSTICK": "RIGHT_STICK",
        "RIGHTTHUMB": "RIGHT_THUMB",
        "RIGHTSTICKBUTTON": "RIGHT_STICK",
        "DPADUP": "DPAD_UP",
        "DPADDOWN": "DPAD_DOWN",
        "DPADRIGHT": "DPAD_RIGHT",
        "DPADLEFT": "DPAD_LEFT",
        "DUP": "DPAD_UP",
        "DDOWN": "DPAD_DOWN",
        "DRIGHT": "DPAD_RIGHT",
        "DLEFT": "DPAD_LEFT",
        "LEFTBUMPER": "LEFT_BUMPER",
        "LEFTSHOULDER": "LEFT_SHOULDER",
        "RIGHTBUMPER": "RIGHT_BUMPER",
        "RIGHTSHOULDER": "RIGHT_SHOULDER",
        "LEFTTRIGGER": "LEFT_TRIGGER",
        "RIGHTTRIGGER": "RIGHT_TRIGGER",
    }
    return aliases.get(compact, compact)


@dataclass
class GamepadMirrorMapper:
    """Maps carb gamepad events to the MuJoCo mirror joystick semantics.

    IsaacLab Se2Gamepad covers analog sticks only. This mapper adds the mirror's
    Hitbox/XInput path: D-pad as planar velocity, LB/RB as yaw, and LT/RT as
    held EE step on the currently selected sphere dimension.
    """

    max_vx: float = 0.3
    max_vy: float = 0.2
    max_wz: float = 0.3
    press_thresh: float = 0.5
    _edges: ButtonEdgeFilter = field(init=False)
    _held: dict[str, bool] = field(init=False)

    def __post_init__(self) -> None:
        self._edges = ButtonEdgeFilter(press_thresh=self.press_thresh)
        self._held = {name: False for name in set(_HELD_ALIASES.values())}

    def update(self, input_name: str, value: float) -> dict[str, object]:
        """Update state from one carb event and return one-shot edge fields."""
        name = _norm(input_name)
        held_name = _HELD_ALIASES.get(name)
        if held_name is not None:
            self._held[held_name] = float(value) >= self.press_thresh
            return {}

        field_name = _EDGE_ALIASES.get(name)
        if field_name is None or not self._edges.update(name, float(value)):
            return {}
        return {field_name: True}

    def resolve_velocity(self, stick_velocity: Iterable[float]) -> tuple[float, float, float]:
        """Return mirror vx/vy/wz, using D-pad/bumpers when sticks are inactive."""
        vx, vy, wz = (float(v) for v in stick_velocity)
        # IsaacLab Se2Gamepad signs: left-stick-right and right-stick-right are positive.
        # MuJoCo mirror signs: right is negative vy, right yaw is negative wz.
        vy = -vy
        wz = -wz

        dpad_y = float(self._held["dpad_up"]) - float(self._held["dpad_down"])
        dpad_x = float(self._held["dpad_right"]) - float(self._held["dpad_left"])
        bumper_yaw = float(self._held["left_bumper"]) - float(self._held["right_bumper"])

        if abs(vx) <= 1e-6:
            vx = dpad_y * float(self.max_vx)
        if abs(vy) <= 1e-6:
            vy = -dpad_x * float(self.max_vy)
        if abs(wz) <= 1e-6:
            wz = bumper_yaw * float(self.max_wz)
        return vx, vy, wz

    def held_command_fields(self) -> dict[str, bool]:
        return {
            "ee_step_positive_held": self._held["right_trigger"],
            "ee_step_negative_held": self._held["left_trigger"],
        }

    def reset(self) -> None:
        self._edges.reset()
        for name in self._held:
            self._held[name] = False
