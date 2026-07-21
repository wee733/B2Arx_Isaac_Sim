from __future__ import annotations

from geometry_msgs.msg import Point32, PolygonStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .behavior_footprint_core import validate_flat_footprint


class BehaviorFootprintPublisher(Node):
    """Publish the configured static footprint directly in the robot frame.

    Nav2's behavior server otherwise transforms the costmap's already-oriented
    footprint using the footprint message timestamp. In this simulation the
    costmap and behavior processes can momentarily disagree on their latest
    official ZED VIO TF sample, causing a future-extrapolation failure.
    Publishing the same configured static geometry in base_link with a zero
    timestamp removes that exact-time cross-process lookup without changing
    ZED timestamps or any Nav2 collision-checking algorithm.
    """

    def __init__(self) -> None:
        super().__init__("behavior_footprint_publisher")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter(
            "points",
            [0.47, 0.31, 0.47, -0.31, -0.47, -0.31, -0.47, 0.31],
        )
        self.declare_parameter("publish_rate_hz", 5.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        if not self._frame_id:
            raise ValueError("behavior footprint frame_id must not be empty")
        self._points = validate_flat_footprint(self.get_parameter("points").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError(
                f"behavior footprint publish_rate_hz must be positive, got {publish_rate_hz}"
            )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(PolygonStamped, "footprint_out", qos)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish,
            clock=self._steady_clock,
        )
        self._publish()
        self.get_logger().info(
            "behavior footprint active: frame=%s points=%d publish_rate=%.1fHz"
            % (self._frame_id, len(self._points), publish_rate_hz)
        )

    def _publish(self) -> None:
        msg = PolygonStamped()
        msg.header.frame_id = self._frame_id
        # A zero timestamp is the standard tf2 request for the latest transform.
        # Geometry is already expressed in base_link, so the transform is identity.
        msg.polygon.points = [
            Point32(x=float(x), y=float(y), z=0.0) for x, y in self._points
        ]
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BehaviorFootprintPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
