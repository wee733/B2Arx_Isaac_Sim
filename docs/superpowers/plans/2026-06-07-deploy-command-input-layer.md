# Deploy Command-Input Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire live keyboard/gamepad input into the B2+ARX R5 deployment FSM through a `CommandSource` abstraction, with all config unified under a single `deploy_config.yaml`.

**Architecture:** A new `scripts/policy_deploy/command_sources/` subpackage turns any input device into one `ArmLocoCommand` per control tick. Pure-logic units (`_CommandLatch`, `ButtonEdgeFilter`, `ScriptedCommandSource`) are unit-tested without Isaac; carb-dependent device wrappers (`KeyboardCommandSource`, `GamepadCommandSource`) lazy-import carb and are covered by Isaac smoke. The controller swaps its static `command` for a `command_source` and passes `stale` to the FSM. `runtime.py` and `fsm.py` stay frozen.

**Tech Stack:** Python 3.11, NumPy, PyYAML, IsaacLab `Se2Keyboard`/`Se2Gamepad` devices, `carb.input` gamepad event subscription, pytest (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` to avoid the ROS jazzy `lark` collection crash).

**Spec:** `docs/superpowers/specs/2026-06-07-deploy-command-input-layer-design.md`

**Working-tree note:** The tree is not clean (README/assets/scripts/tests have pre-existing edits and untracked files). Build on top of the current working tree — do NOT roll back or revert existing changes.

**Test invocation (all unit-test steps):**
```bash
source /home/lbz/miniforge3/etc/profile.d/conda.sh && conda activate isaaclab
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py
```

---

## File Structure

| File | Responsibility |
|---|---|
| `scripts/policy_deploy/command_sources/__init__.py` | Package exports + `make_command_source(input_settings, deploy_settings)` factory (lazy carb import) |
| `scripts/policy_deploy/command_sources/base.py` | `CommandSource(ABC)` + `ScriptedCommandSource` |
| `scripts/policy_deploy/command_sources/latch.py` | `_CommandLatch` — stores one-shot edge events, cleared on read |
| `scripts/policy_deploy/command_sources/edge.py` | `ButtonEdgeFilter` — rising-edge detection for analog gamepad buttons |
| `scripts/policy_deploy/command_sources/devices.py` | `KeyboardCommandSource`, `GamepadCommandSource` (wrap IsaacLab devices) |
| `scripts/policy_deploy/deploy_config.py` | MODIFY: add `InputSettings`, make it `DeployConfig.input` |
| `scripts/policy_deploy/isaac_controller.py` | MODIFY: `command` → `command_source`; pass `stale`; `close()` |
| `scripts/isaac_b2arx_scene.py` | MODIFY: collapse CLI to `--deploy_config`; rewrite `make_policy_controller`; `close()` in finally |
| `scripts/policy_deploy/deploy_config.example.yaml` | CREATE: real runnable example config |
| `tests/test_command_sources.py` | CREATE: pure-Python unit tests |
| `tests/test_deploy_config.py` | CREATE: config-parsing unit tests |

**Old `scripts/policy_deploy/input/` (docstring-only stub) is deleted in Task 1.**

---
### Task 1: Latch + edge filter (pure logic, TDD)

**Files:**
- Delete: `scripts/policy_deploy/input/` (docstring-only stub)
- Create: `scripts/policy_deploy/command_sources/__init__.py` (empty for now)
- Create: `scripts/policy_deploy/command_sources/latch.py`
- Create: `scripts/policy_deploy/command_sources/edge.py`
- Create: `tests/test_command_sources.py`

- [ ] **Step 1: Remove the old stub and create the package dir**

```bash
git rm -r scripts/policy_deploy/input
mkdir -p scripts/policy_deploy/command_sources
printf '"""Unified deployment command-input layer: device -> ArmLocoCommand."""\n' > scripts/policy_deploy/command_sources/__init__.py
```

- [ ] **Step 2: Write failing tests for `_CommandLatch` and `ButtonEdgeFilter`**

Create `tests/test_command_sources.py`:

```python
from __future__ import annotations

from scripts.policy_deploy.command_sources.latch import _CommandLatch
from scripts.policy_deploy.command_sources.edge import ButtonEdgeFilter


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
```

- [ ] **Step 3: Run tests, verify they fail**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py`
Expected: FAIL with `ModuleNotFoundError: scripts.policy_deploy.command_sources.latch`

- [ ] **Step 4: Implement `latch.py`**

```python
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
```

- [ ] **Step 5: Implement `edge.py`**

```python
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
```

- [ ] **Step 6: Run tests, verify they pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py`
Expected: PASS (5 tests)

- [ ] **Step 7: Commit**

```bash
git add scripts/policy_deploy/command_sources tests/test_command_sources.py
git rm -r --cached scripts/policy_deploy/input 2>/dev/null || true
git commit -m "feat(deploy): add command-source latch and edge filter"
```

---
### Task 2: CommandSource base + ScriptedCommandSource (TDD)

**Files:**
- Create: `scripts/policy_deploy/command_sources/base.py`
- Modify: `tests/test_command_sources.py`

**Mutation safety (spec + review):** `poll()` MUST return a fresh `ArmLocoCommand`
each tick. The controller overlays auto_arm_loco onto the returned command; a reused
object would leak one-shot edges into the next tick.

- [ ] **Step 1: Add failing tests for `ScriptedCommandSource`**

Append to `tests/test_command_sources.py`:

```python
from scripts.policy_deploy.command_sources.base import ScriptedCommandSource


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
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py`
Expected: FAIL with `ModuleNotFoundError: ...command_sources.base`

- [ ] **Step 3: Implement `base.py`**

```python
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
```

- [ ] **Step 4: Run tests, verify they pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py`
Expected: PASS (8 tests)

- [ ] **Step 5: Commit**

```bash
git add scripts/policy_deploy/command_sources/base.py tests/test_command_sources.py
git commit -m "feat(deploy): add CommandSource ABC and ScriptedCommandSource"
```

---
### Task 3: InputSettings in deploy_config.py (TDD)

**Files:**
- Modify: `scripts/policy_deploy/deploy_config.py`
- Create: `tests/test_deploy_config.py`

**Change:** Add an `InputSettings` dataclass and make it `DeployConfig.input` (the 4th
field). Remove the flat `DeploySettings.input_backend` field (migrated to
`InputSettings.backend`). Field names match `Se2KeyboardCfg`/`Se2GamepadCfg`:
`v_x_sensitivity` (0.8 kbd / 1.0 pad), `v_y_sensitivity` (0.4 / 1.0),
`omega_z_sensitivity` (1.0), `dead_zone` (0.01, gamepad only).

- [ ] **Step 1: Write failing tests**

Create `tests/test_deploy_config.py`:

```python
from __future__ import annotations

import yaml

from scripts.policy_deploy.deploy_config import DeployConfig, InputSettings


def test_input_settings_defaults() -> None:
    cfg = DeployConfig.from_dict({})
    assert cfg.input.backend == "scripted"
    assert cfg.input.keyboard["v_x_sensitivity"] == 0.8
    assert cfg.input.keyboard["v_y_sensitivity"] == 0.4
    assert cfg.input.keyboard["omega_z_sensitivity"] == 1.0
    assert cfg.input.gamepad["dead_zone"] == 0.01


def test_input_settings_parsed_from_yaml() -> None:
    data = yaml.safe_load(
        """
        input:
          backend: keyboard
          keyboard:
            v_x_sensitivity: 1.5
          gamepad:
            dead_zone: 0.05
        """
    )
    cfg = DeployConfig.from_dict(data)
    assert cfg.input.backend == "keyboard"
    # explicit override wins; unspecified keys fall back to defaults
    assert cfg.input.keyboard["v_x_sensitivity"] == 1.5
    assert cfg.input.keyboard["v_y_sensitivity"] == 0.4
    assert cfg.input.gamepad["dead_zone"] == 0.05


def test_deploy_settings_no_longer_has_input_backend() -> None:
    cfg = DeployConfig.from_dict({"deploy": {"start_state": "ArmLoco"}})
    assert not hasattr(cfg.deploy, "input_backend")
    assert cfg.deploy.start_state == "ArmLoco"
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_deploy_config.py`
Expected: FAIL with `ImportError: cannot import name 'InputSettings'`

- [ ] **Step 3: Add `InputSettings` to `deploy_config.py`**

Insert after the `DeploySettings` class (before `DeployConfig`):

```python
_KEYBOARD_DEFAULTS = {"v_x_sensitivity": 0.8, "v_y_sensitivity": 0.4, "omega_z_sensitivity": 1.0}
_GAMEPAD_DEFAULTS = {
    "v_x_sensitivity": 1.0,
    "v_y_sensitivity": 1.0,
    "omega_z_sensitivity": 1.0,
    "dead_zone": 0.01,
}


@dataclass
class InputSettings:
    """Live-input backend selection plus per-device sensitivities.

    Field names align with IsaacLab Se2KeyboardCfg / Se2GamepadCfg so values pass
    straight through to the device cfg.
    """

    backend: str = "scripted"  # scripted / keyboard / gamepad
    keyboard: dict = field(default_factory=lambda: dict(_KEYBOARD_DEFAULTS))
    gamepad: dict = field(default_factory=lambda: dict(_GAMEPAD_DEFAULTS))

    @classmethod
    def from_dict(cls, data: dict) -> "InputSettings":
        data = data or {}
        kbd = dict(_KEYBOARD_DEFAULTS)
        kbd.update(data.get("keyboard") or {})
        pad = dict(_GAMEPAD_DEFAULTS)
        pad.update(data.get("gamepad") or {})
        return cls(backend=str(data.get("backend", "scripted")), keyboard=kbd, gamepad=pad)
```

- [ ] **Step 4: Remove `input_backend` from `DeploySettings`**

In `DeploySettings`, delete the line `input_backend: str = "keyboard"` and, in its
`from_dict`, delete the `input_backend=str(data.get("input_backend", "keyboard")),`
argument. Leave the rest of `DeploySettings` unchanged.

- [ ] **Step 5: Add `input` field to `DeployConfig`**

In `DeployConfig`, add the field after `deploy`:

```python
    input: InputSettings = field(default_factory=InputSettings)
```

And in `DeployConfig.from_dict`, add to the returned `cls(...)`:

```python
            input=InputSettings.from_dict(data.get("input") or {}),
```

- [ ] **Step 6: Run tests, verify they pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_deploy_config.py`
Expected: PASS (3 tests)

- [ ] **Step 7: Commit**

```bash
git add scripts/policy_deploy/deploy_config.py tests/test_deploy_config.py
git commit -m "feat(deploy): add InputSettings to deploy config"
```

---
### Task 4: Device sources + factory (carb lazy-import, smoke-covered)

**Files:**
- Create: `scripts/policy_deploy/command_sources/devices.py`
- Modify: `scripts/policy_deploy/command_sources/__init__.py`
- Modify: `tests/test_command_sources.py` (factory tests only — no carb)

Keyboard/gamepad construction needs a running `simulation_app`, so these are NOT
unit-tested; the factory's scripted path and error handling ARE.

- [ ] **Step 1: Add failing factory tests**

Append to `tests/test_command_sources.py`:

```python
import pytest

from scripts.policy_deploy.command_sources import make_command_source
from scripts.policy_deploy.command_sources.base import ScriptedCommandSource
from scripts.policy_deploy.deploy_config import DeployConfig


def test_factory_scripted_does_not_import_carb() -> None:
    import sys
    cfg = DeployConfig.from_dict({"deploy": {"command": [0.1, 0.2, 0.3]},
                                  "input": {"backend": "scripted"}})
    src = make_command_source(cfg.input, cfg.deploy)
    assert isinstance(src, ScriptedCommandSource)
    assert src.poll().vx == 0.1
    assert "carb" not in sys.modules  # scripted path must stay carb-free


def test_factory_rejects_unknown_backend() -> None:
    cfg = DeployConfig.from_dict({"input": {"backend": "joystick3000"}})
    with pytest.raises(ValueError, match="joystick3000"):
        make_command_source(cfg.input, cfg.deploy)
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py`
Expected: FAIL with `ImportError: cannot import name 'make_command_source'`

- [ ] **Step 3: Write `__init__.py` factory (lazy carb import)**

Overwrite `scripts/policy_deploy/command_sources/__init__.py`:

```python
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
```

- [ ] **Step 4: Run factory tests, verify they pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py`
Expected: PASS (10 tests)

- [ ] **Step 5: Commit the factory**

```bash
git add scripts/policy_deploy/command_sources/__init__.py tests/test_command_sources.py
git commit -m "feat(deploy): add command-source factory with scripted fallback"
```

---
### Task 5: KeyboardCommandSource (devices.py, carb)

**Files:**
- Create: `scripts/policy_deploy/command_sources/devices.py`

No unit test (needs `simulation_app`); verified by Isaac smoke in Task 8. Keyboard
state-transition keys avoid IsaacLab's reserved keys (L=reset, Z/X=yaw, arrows=velocity):
`F`=FixStand `G`=ArmPreAlign `H`=ArmLoco `P`=Passive `R`=EE cycle dim `I`/`K`=EE +/-
`O`=EE reset. Velocity reuses `Se2Keyboard.advance()` (arrows/numpad + Z/X).

- [ ] **Step 1: Write `devices.py` with `KeyboardCommandSource`**

```python
from __future__ import annotations

import carb
import carb.input
from isaaclab.devices.keyboard.se2_keyboard import Se2Keyboard, Se2KeyboardCfg
from isaaclab.devices.gamepad.se2_gamepad import Se2Gamepad, Se2GamepadCfg

from ..fsm import ArmLocoCommand
from .edge import ButtonEdgeFilter
from .latch import _CommandLatch

# Keyboard letter -> latch event name (one-shot, clean KEY_PRESS edges).
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
```

- [ ] **Step 2: Byte-compile to catch syntax errors**

Run: `python -m py_compile scripts/policy_deploy/command_sources/devices.py`
Expected: exit 0 (imports are not executed; carb is only touched at runtime under Isaac).

- [ ] **Step 3: Commit**

```bash
git add scripts/policy_deploy/command_sources/devices.py
git commit -m "feat(deploy): add KeyboardCommandSource over Se2Keyboard"
```

---
### Task 6: GamepadCommandSource (devices.py, self-subscribed carb)

**Files:**
- Modify: `scripts/policy_deploy/command_sources/devices.py`

`Se2Gamepad.add_callback()` fires `func()` with NO args, so it can't see
`event.value` and can't distinguish press from release. Therefore buttons are handled
by a SECOND carb subscription this class owns; velocity still uses
`Se2Gamepad.advance()`. The button sub is released in `close()` (and `__del__`).
carb button enum names (verified): `A B X Y MENU LEFT_STICK RIGHT_STICK LEFT_TRIGGER
RIGHT_TRIGGER`. Mapping: `A`=FixStand `LEFT_STICK`(thumb)=ArmPreAlign `Y`/`MENU`=ArmLoco
`B`=Passive `X`=EE cycle `RIGHT_STICK`(thumb)=EE reset; `RIGHT_TRIGGER`=EE+ `LEFT_TRIGGER`=EE-.

- [ ] **Step 1: Append `GamepadCommandSource` to `devices.py`**

```python
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


class GamepadCommandSource:
    """Wraps Se2Gamepad for velocity; self-subscribes carb for edge-filtered buttons."""

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
```

- [ ] **Step 2: Byte-compile**

Run: `python -m py_compile scripts/policy_deploy/command_sources/devices.py`
Expected: exit 0.

- [ ] **Step 3: Commit**

```bash
git add scripts/policy_deploy/command_sources/devices.py
git commit -m "feat(deploy): add GamepadCommandSource with carb edge-filtered buttons"
```

---
### Task 7: Wire controller to command_source

**Files:**
- Modify: `scripts/policy_deploy/isaac_controller.py`

Swap the static `command: ArmLocoCommand` constructor arg for `command_source:
CommandSource`. `_command_for_current_state()` polls the source instead of copying a
static command; auto_arm_loco overlay is unchanged. Pass `stale` to the FSM. Add `close()`.
No unit test (the controller needs an Isaac robot); covered by smoke in Task 8. The
existing 12 tests in `test_policy_deploy.py` construct FSM states directly, not
`B2ArxIsaacPolicyController`, so they remain green.

- [ ] **Step 1: Update imports**

In `isaac_controller.py`, add to the `.fsm` import block (top of file) — add
`ArmLocoCommand` is already imported; add a new import line:

```python
from .command_sources import CommandSource, ScriptedCommandSource
```

- [ ] **Step 2: Change the constructor signature**

In `B2ArxIsaacPolicyController.__init__`, replace the parameter
`command: ArmLocoCommand | None = None,` with:

```python
        command_source: CommandSource | None = None,
```

And replace the body line `self.command = command or ArmLocoCommand()` with:

```python
        self.command_source = command_source or ScriptedCommandSource()
```

- [ ] **Step 3: Poll the source in `_command_for_current_state`**

Replace the first line of `_command_for_current_state` —
`cmd = ArmLocoCommand(**self.command.__dict__)` — with:

```python
        cmd = self.command_source.poll()
```

`poll()` already returns a fresh `ArmLocoCommand`, so the auto_arm_loco overlay below
mutates a per-tick object (no leakage). The rest of the method is unchanged.

- [ ] **Step 4: Pass `stale` to the FSM in `_tick_control`**

In `_tick_control`, the call is currently:

```python
        q_target = self.fsm.tick(self.plant, command, tilt_rad=state.tilt_rad, stale=False, t=self._state_elapsed)
```

Replace `stale=False` with `stale=self.command_source.is_stale()`:

```python
        q_target = self.fsm.tick(self.plant, command, tilt_rad=state.tilt_rad, stale=self.command_source.is_stale(), t=self._state_elapsed)
```

- [ ] **Step 5: Add `close()` to the controller**

Add a method to `B2ArxIsaacPolicyController` (after `reset`):

```python
    def close(self) -> None:
        """Release the command source (carb subscriptions)."""
        self.command_source.close()
```

- [ ] **Step 6: Verify existing tests still pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_policy_deploy.py`
Expected: PASS (12 tests) — the controller change does not touch FSM/runtime contracts.

- [ ] **Step 7: Byte-compile**

Run: `python -m py_compile scripts/policy_deploy/isaac_controller.py`
Expected: exit 0.

- [ ] **Step 8: Commit**

```bash
git add scripts/policy_deploy/isaac_controller.py
git commit -m "feat(deploy): drive controller from CommandSource with staleness + close"
```

---
### Task 8: Scene CLI → --deploy_config + example yaml

**Files:**
- Modify: `scripts/isaac_b2arx_scene.py`
- Create: `scripts/policy_deploy/deploy_config.example.yaml`

Collapse the six `--policy_*` switches to a single `--deploy_config` defaulting to a
real runnable example. Rewrite `make_policy_controller` to load the config and build a
command source. Call `controller.close()` in a `finally` so carb subs are released on
error, window close, or duration end.

- [ ] **Step 1: Create the example config**

Create `scripts/policy_deploy/deploy_config.example.yaml`:

```yaml
# Runnable deploy config for --control_mode policy. Direct-ArmLoco smoke default.
policy:
  run_dir: /home/lbz/b2arx/b2arx_sim2real_v1/logs/rsl_rl/b2arx_direct/2026-06-07_02-01-02
  # onnx / deploy_yaml: optional overrides; default to run_dir/exported|params layout.

deploy:
  start_state: ArmLoco        # Passive / FixStand / ArmPreAlign / ArmLoco
  auto_arm_loco: false
  ee_sphere: [0.36, 0.56, 0.0]
  command: [0.0, 0.0, 0.0]    # scripted backend fixed vx/vy/wz

input:
  backend: scripted           # scripted / keyboard / gamepad
  keyboard:
    v_x_sensitivity: 0.8
    v_y_sensitivity: 0.4
    omega_z_sensitivity: 1.0
  gamepad:
    v_x_sensitivity: 1.0
    v_y_sensitivity: 1.0
    omega_z_sensitivity: 1.0
    dead_zone: 0.01
```

- [ ] **Step 2: Replace the `--policy_*` argparse block**

In `scripts/isaac_b2arx_scene.py`, delete these arguments: `--policy_onnx`,
`--policy_deploy_yaml`, `--policy_start_state`, `--policy_command`, `--policy_ee_sphere`,
`--policy_auto_arm_loco`. Keep `--control_mode` and `--print_policy_debug`. In their
place add:

```python
DEFAULT_DEPLOY_CONFIG = Path(__file__).resolve().parent / "policy_deploy" / "deploy_config.example.yaml"
parser.add_argument(
    "--deploy_config",
    type=str,
    default=str(DEFAULT_DEPLOY_CONFIG),
    help="deploy_config.yaml for --control_mode policy (policy/scene/deploy/input).",
)
```

- [ ] **Step 3: Update the top-of-file import**

Replace `from policy_deploy.fsm import ArmLocoCommand` with:

```python
from policy_deploy.command_sources import make_command_source
from policy_deploy.deploy_config import load_deploy_config
```

(Keep `from policy_deploy.isaac_controller import B2ArxIsaacPolicyController`.)

- [ ] **Step 4: Rewrite `make_policy_controller`**

Replace the whole `make_policy_controller` function with:

```python
def make_policy_controller(robot) -> B2ArxIsaacPolicyController:
    cfg = load_deploy_config(args_cli.deploy_config)
    onnx = cfg.policy.resolved_onnx()
    deploy_yaml = cfg.policy.resolved_deploy_yaml()
    if not Path(onnx).exists():
        raise FileNotFoundError(f"Policy ONNX not found: {onnx}")
    if not Path(deploy_yaml).exists():
        raise FileNotFoundError(f"Policy deploy.yaml not found: {deploy_yaml}")
    source = make_command_source(cfg.input, cfg.deploy)
    controller = B2ArxIsaacPolicyController(
        robot,
        deploy_yaml=deploy_yaml,
        onnx_path=onnx,
        start_state=cfg.deploy.start_state,
        command_source=source,
        ee_sphere=cfg.deploy.ee_sphere,
        auto_arm_loco=cfg.deploy.auto_arm_loco,
    )
    controller.reset()
    print(
        "[INFO]: Policy controller loaded: "
        f"config={args_cli.deploy_config} backend={cfg.input.backend} "
        f"start_state={cfg.deploy.start_state} auto_arm_loco={cfg.deploy.auto_arm_loco} "
        f"control_dt={controller.control_dt:.4f}s",
        flush=True,
    )
    return controller
```

- [ ] **Step 5: Close the controller in a `finally`**

In `run_simulator`, wrap the `while simulation_app.is_running():` loop so the controller
is always closed. Change the structure to:

```python
    try:
        while simulation_app.is_running():
            ...  # existing loop body unchanged
    finally:
        if policy_controller is not None:
            policy_controller.close()
```

(Indent the existing loop body under `try:`; do not change its logic.)

- [ ] **Step 6: Byte-compile**

Run: `python -m py_compile scripts/isaac_b2arx_scene.py`
Expected: exit 0.

- [ ] **Step 7: Commit**

```bash
git add scripts/isaac_b2arx_scene.py scripts/policy_deploy/deploy_config.example.yaml
git commit -m "feat(deploy): collapse policy CLI to --deploy_config with example"
```

---
### Task 9: README update + smoke verification

**Files:**
- Modify: `README.md` (lines ~57-90, the three policy command blocks)

- [ ] **Step 1: Replace the policy command examples in README**

Replace the three policy `bash` blocks (README lines ~57-90, "直接从 ArmLoco 启动策略"
through the "手动指定策略文件" block) with config-driven equivalents:

````markdown
直接从 `ArmLoco` 启动策略（默认 example 配置即为此场景）：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --control_mode policy \
  --print_policy_debug
```

走完整自动 FSM 或自定义策略/速度/EE/输入设备：复制
`scripts/policy_deploy/deploy_config.example.yaml`，改 `deploy.start_state`、
`deploy.auto_arm_loco`、`deploy.command`、`deploy.ee_sphere`、`input.backend`
（scripted / keyboard / gamepad），再用 `--deploy_config <你的.yaml>` 指定：

```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --enable_cameras \
  --control_mode policy \
  --deploy_config /path/to/my_deploy_config.yaml \
  --duration 5.0 \
  --print_policy_debug
```

键盘遥控键位：`F`=FixStand `G`=ArmPreAlign `H`=ArmLoco `P`=Passive；
`R`=切换 EE 维度 `I`/`K`=当前维 ± `O`=EE 复位；方向键/小键盘走 vx/vy，`Z`/`X` 走 yaw。
````

Keep the existing note about `FixStand` 3s / `ArmPreAlign` 0.5s timing.

- [ ] **Step 2: Commit the docs**

```bash
git add README.md
git commit -m "docs: switch policy deployment to --deploy_config workflow"
```

- [ ] **Step 3: Full unit-test sweep**

Run:
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_command_sources.py tests/test_deploy_config.py tests/test_policy_deploy.py
```
Expected: PASS (10 + 3 + 12 = 25 tests).

- [ ] **Step 4: Byte-compile the whole package**

Run:
```bash
python -m py_compile scripts/isaac_b2arx_scene.py scripts/policy_deploy/*.py scripts/policy_deploy/command_sources/*.py
```
Expected: exit 0.

- [ ] **Step 5: Smoke — direct ArmLoco (scripted backend)**

Run:
```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --headless --enable_cameras --control_mode policy --duration 1.05 --print_policy_debug
```
Expected: controller loads (`backend=scripted start_state=ArmLoco`); at least one
`[POLICY]:` line prints with `state=ArmLoco` and a finite non-zero `raw_abs_max`; loop
finishes cleanly. (Diagnostics print every ~1.0s, so 1.05s captures one post-step frame.)

- [ ] **Step 6: Smoke — full FSM (auto_arm_loco)**

Create `/tmp/auto_deploy.yaml` copying the example but with
`deploy: {start_state: Passive, auto_arm_loco: true}`. Run:
```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --headless --enable_cameras --control_mode policy --deploy_config /tmp/auto_deploy.yaml \
  --duration 5.0 --print_policy_debug
```
Expected: FSM walks Passive→FixStand→ArmPreAlign→ArmLoco; final `[POLICY]:` line shows
`state=ArmLoco`; loop finishes cleanly.

- [ ] **Step 7: Smoke — hold-mode regression**

Run:
```bash
TERM=xterm /home/lbz/IsaacLab/isaaclab.sh -p scripts/isaac_b2arx_scene.py \
  --headless --enable_cameras --control_mode hold --duration 0.1
```
Expected: scene runs and finishes; policy controller is never constructed (no
`--deploy_config` load, no carb input). Confirms hold mode is unaffected.

---

## Notes for the implementer

- **Frozen contracts:** Do NOT edit `runtime.py` or `fsm.py`. If a change seems to
  require it, stop and re-read the spec — the policy-deployment contract is fixed.
- **carb is runtime-only:** `command_sources/devices.py` imports carb/omni at module
  top, but that module is only imported by the factory's keyboard/gamepad branch, which
  runs under a live `simulation_app`. `py_compile` checks syntax without importing.
- **Keyboard/gamepad have no unit tests** by design; their correctness is established by
  the Isaac smoke runs (Tasks 8-9). Only pure-logic units are unit-tested.
- **Manual device check** (optional, needs a windowed session + real devices): launch
  non-headless with `input.backend: keyboard` and confirm F/G/H/P switch states and
  arrow keys drive velocity; with a gamepad attached and `backend: gamepad`, confirm A/B
  switch states and sticks drive velocity.
