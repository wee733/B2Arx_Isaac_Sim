from __future__ import annotations

import math
import time
from collections.abc import Callable

from .base import CommandSource
from ..fsm import ArmLocoCommand


CMD_VEL_GRAPH_PATH = "/World/B2ArxROS2CmdVelGraph"
CMD_VEL_SUBSCRIBER_NODE = f"{CMD_VEL_GRAPH_PATH}/SubscribeTwist"
CMD_VEL_HEARTBEAT_NODE = f"{CMD_VEL_GRAPH_PATH}/SubscribeHeartbeat"


def _positive_setting(settings: dict, name: str) -> float:
    value = float(settings[name])
    if value <= 0.0:
        raise ValueError(f"ros2_twist.{name} must be positive, got {value}")
    return value


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def adapt_nav2_velocity(
    vx: float,
    vy: float,
    wz: float,
    *,
    min_planar_speed: float = 0.25,
    min_turn_radius: float = 0.42,
    zero_epsilon: float = 0.01,
) -> tuple[float, float, float]:
    """Project a Nav2 Twist into model_29999's trained command domain.

    The policy command is a body-frame yaw-rate target, not an absolute yaw.
    Training contained either a full stop or a planar command of at least
    ``min_planar_speed``; pure rotation was absent.  Reject pure rotation
    rather than inventing an unplanned forward displacement, then preserve the
    requested planar direction and cap curvature for the external path tracker.
    """
    min_planar_speed = float(min_planar_speed)
    min_turn_radius = float(min_turn_radius)
    zero_epsilon = float(zero_epsilon)
    if min_planar_speed <= 0.0:
        raise ValueError(f"min_planar_speed must be positive, got {min_planar_speed}")
    if min_turn_radius <= 0.0:
        raise ValueError(f"min_turn_radius must be positive, got {min_turn_radius}")
    if zero_epsilon <= 0.0:
        raise ValueError(f"zero_epsilon must be positive, got {zero_epsilon}")

    vx = float(vx)
    vy = float(vy)
    wz = float(wz)
    speed = math.hypot(vx, vy)
    if speed <= zero_epsilon:
        return 0.0, 0.0, 0.0
    if speed < min_planar_speed:
        scale = min_planar_speed / speed
        vx *= scale
        vy *= scale
        speed = min_planar_speed
    wz = _clip(wz, speed / min_turn_radius)
    return vx, vy, wz


class OmniGraphTwistReader:
    """Read the official Twist subscriber plus explicit UInt32 heartbeat."""

    def __init__(self) -> None:
        import omni.graph.core as og

        self._og = og
        subscriber = og.Controller.node(CMD_VEL_SUBSCRIBER_NODE)
        heartbeat = og.Controller.node(CMD_VEL_HEARTBEAT_NODE)
        if not subscriber.is_valid() or not heartbeat.is_valid():
            raise RuntimeError(
                "ROS 2 cmd_vel graph is missing. Launch the scene with --nav2 "
                f"(expected {CMD_VEL_GRAPH_PATH})."
            )
        self._linear = og.Controller.attribute("outputs:linearVelocity", subscriber)
        self._angular = og.Controller.attribute("outputs:angularVelocity", subscriber)
        self._heartbeat = og.Controller.attribute("outputs:data", heartbeat)

    def read(self) -> tuple[int, tuple[float, float, float], tuple[float, float, float]]:
        linear = tuple(float(value) for value in self._og.Controller.get(self._linear))
        angular = tuple(float(value) for value in self._og.Controller.get(self._angular))
        return int(self._og.Controller.get(self._heartbeat)), linear, angular


class Ros2TwistCommandSource(CommandSource):
    """Convert Nav2 geometry_msgs/Twist into the existing locomotion policy contract."""

    def __init__(
        self,
        settings: dict,
        *,
        reader=None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.topic = str(settings.get("topic", "/cmd_vel"))
        if not self.topic.startswith("/"):
            raise ValueError(f"ros2_twist.topic must be absolute, got {self.topic!r}")
        self.heartbeat_topic = str(settings.get("heartbeat_topic", "/cmd_vel_heartbeat"))
        if not self.heartbeat_topic.startswith("/"):
            raise ValueError(
                "ros2_twist.heartbeat_topic must be absolute, "
                f"got {self.heartbeat_topic!r}"
        )
        self.timeout_s = _positive_setting(settings, "timeout_s")
        self.passive_timeout_s = _positive_setting(settings, "passive_timeout_s")
        if self.passive_timeout_s < self.timeout_s:
            raise ValueError(
                "ros2_twist.passive_timeout_s must be greater than or equal to "
                f"timeout_s ({self.timeout_s}), got {self.passive_timeout_s}"
            )
        self.max_vx = _positive_setting(settings, "max_vx")
        self.max_vy = _positive_setting(settings, "max_vy")
        self.max_wz = _positive_setting(settings, "max_wz")
        self.min_planar_speed = float(settings.get("min_planar_speed", 0.25))
        self.min_turn_radius = float(settings.get("min_turn_radius", 0.42))
        self.zero_epsilon = float(settings.get("zero_epsilon", 0.01))
        # Validate the adapter contract during startup instead of waiting for
        # the first navigation command.
        adapt_nav2_velocity(
            0.0,
            0.0,
            0.0,
            min_planar_speed=self.min_planar_speed,
            min_turn_radius=self.min_turn_radius,
            zero_epsilon=self.zero_epsilon,
        )
        self._reader = reader if reader is not None else OmniGraphTwistReader()
        self._monotonic = monotonic
        self._last_count = 0
        self._last_rx_time: float | None = None
        self._received_heartbeat = False
        self._stale_reference_time = 0.0
        self._command = (0.0, 0.0, 0.0)
        self.reset()

    def reset(self) -> None:
        count, _, _ = self._reader.read()
        self._last_count = count
        self._last_rx_time = None
        self._received_heartbeat = False
        self._stale_reference_time = self._monotonic()
        self._command = (0.0, 0.0, 0.0)

    def poll(self) -> ArmLocoCommand:
        count, linear, angular = self._reader.read()
        now = self._monotonic()
        if count != self._last_count:
            self._last_count = count
            self._last_rx_time = now
            self._received_heartbeat = True
            self._stale_reference_time = now
            self._command = adapt_nav2_velocity(
                _clip(linear[0], self.max_vx),
                _clip(linear[1], self.max_vy),
                _clip(angular[2], self.max_wz),
                min_planar_speed=self.min_planar_speed,
                min_turn_radius=self.min_turn_radius,
                zero_epsilon=self.zero_epsilon,
            )
        if self._velocity_timed_out(now):
            return ArmLocoCommand()
        vx, vy, wz = self._command
        return ArmLocoCommand(vx=vx, vy=vy, wz=wz)

    def is_stale(self, now_s: float | None = None) -> bool:
        """Report prolonged communications loss to the locomotion FSM.

        A shorter velocity timeout is handled separately in ``poll()`` so a
        normal Nav2 planning/recovery gap commands a safe standstill without
        restarting Passive -> FixStand -> ArmPreAlign -> ArmLoco.

        Isaac Sim and the ZED/ROS bringup are separate processes, so there is
        no finite safe timeout for the first heartbeat.  Until that handshake
        occurs, ``poll()`` keeps velocity at zero and the standing policy stays
        active.  Once one heartbeat has arrived, communication loss uses the
        normal finite timeout and drives the FSM to Passive.
        """
        if not self._received_heartbeat:
            return False
        now = self._monotonic() if now_s is None else float(now_s)
        return now - self._stale_reference_time > self.passive_timeout_s

    def _velocity_timed_out(self, now_s: float) -> bool:
        if self._last_rx_time is None:
            return True
        return float(now_s) - self._last_rx_time > self.timeout_s

    def close(self) -> None:
        self._last_rx_time = None
        self._received_heartbeat = False
        self._stale_reference_time = self._monotonic()
        self._command = (0.0, 0.0, 0.0)
