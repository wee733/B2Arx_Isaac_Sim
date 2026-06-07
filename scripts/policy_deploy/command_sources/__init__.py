"""Unified deployment command-input layer: device -> ArmLocoCommand."""
from __future__ import annotations

from .base import CommandSource, ScriptedCommandSource

__all__ = ["CommandSource", "ScriptedCommandSource", "make_command_source"]

_DEVICE_BACKENDS = ("keyboard", "gamepad")


def make_command_source(input_settings, deploy_settings) -> CommandSource:
    """Route input_settings.backend to a CommandSource.

    scripted -> ScriptedCommandSource (no carb). keyboard/gamepad lazy-import
    devices.py (carb) and fall back to scripted when no gamepad is attached.
    """
    backend = input_settings.backend
    if backend == "scripted":
        return ScriptedCommandSource(command=deploy_settings.command)
    if backend not in _DEVICE_BACKENDS:
        raise ValueError(
            f"unknown input backend {backend!r}; expected one of "
            f"scripted, {', '.join(_DEVICE_BACKENDS)}"
        )
    # Lazy import: only touch carb/omni when a real device is requested.
    from .devices import GamepadCommandSource, KeyboardCommandSource

    if backend == "keyboard":
        return KeyboardCommandSource(input_settings.keyboard)
    gamepad = GamepadCommandSource.try_create(input_settings.gamepad)
    if gamepad is None:
        print("[WARN] gamepad unavailable, falling back to scripted command source", flush=True)
        return ScriptedCommandSource(command=deploy_settings.command)
    return gamepad
