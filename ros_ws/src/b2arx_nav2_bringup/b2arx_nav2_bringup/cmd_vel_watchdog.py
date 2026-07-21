from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import UInt32

from .cmd_vel_watchdog_core import CommandHold


class CmdVelWatchdog(Node):
    """Continuously expose a fail-zero Twist contract to the Isaac policy.

    Nav2's official velocity smoother intentionally stops publishing after it
    reaches zero.  The locomotion policy, however, needs to distinguish that
    normal idle state from loss of the ROS process.  This transport adapter
    republishes fresh smoother commands and emits zero at a fixed rate once
    the input command is old.  It does not alter any non-stale Nav2 command.
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        timeout_s = float(self.get_parameter("input_timeout_s").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be positive, got {publish_rate_hz}")

        self._hold = CommandHold(timeout_s)
        self._publisher = self.create_publisher(Twist, "cmd_vel_out", 1)
        self._heartbeat_publisher = self.create_publisher(UInt32, "heartbeat_out", 1)
        self._subscription = self.create_subscription(Twist, "cmd_vel_in", self._on_command, 1)
        self._heartbeat_sequence = 0

        # Safety timeout and cadence must not depend on simulated /clock.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_command,
            clock=self._steady_clock,
        )
        self.get_logger().info(
            "cmd_vel watchdog active: input_timeout=%.3fs publish_rate=%.1fHz"
            % (timeout_s, publish_rate_hz)
        )

    def _on_command(self, msg: Twist) -> None:
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        try:
            self._hold.update(values, time.monotonic())
        except ValueError as exc:
            self._hold.invalidate()
            self.get_logger().error(f"Ignoring invalid cmd_vel input: {exc}")

    def _publish_command(self) -> None:
        lx, ly, lz, ax, ay, az = self._hold.sample(time.monotonic())
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.linear.z = lz
        msg.angular.x = ax
        msg.angular.y = ay
        msg.angular.z = az
        self._publisher.publish(msg)

        # The sequence is deliberately separate from Twist so fresh Nav2
        # commands remain byte-for-byte unchanged.  A changing value also
        # avoids relying on value-change behavior in the specialized Isaac
        # SubscribeTwist node when the robot is idling at repeated zero.
        self._heartbeat_sequence = (self._heartbeat_sequence + 1) % (2**32)
        heartbeat = UInt32()
        heartbeat.data = self._heartbeat_sequence
        self._heartbeat_publisher.publish(heartbeat)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
