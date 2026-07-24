from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from .odometry_adapter_core import (
    DEFAULT_CAMERA_TO_BASE_QUATERNION,
    DEFAULT_CAMERA_TO_BASE_TRANSLATION,
    OdometryState,
    transform_camera_odometry_to_base,
)


class OdometryAdapter(Node):
    """Expose ZED camera-origin odometry at the B2 ``base_link`` origin."""

    def __init__(self) -> None:
        super().__init__("odometry_adapter")
        self.declare_parameter("input_topic", "/zed/zed_node/odom")
        self.declare_parameter("output_topic", "/b2/odom")
        self.declare_parameter("expected_camera_child_frame_id", "zed_camera_link")
        self.declare_parameter("base_child_frame_id", "base_link")
        self.declare_parameter(
            "camera_to_base_translation",
            list(DEFAULT_CAMERA_TO_BASE_TRANSLATION),
        )
        self.declare_parameter(
            "camera_to_base_quaternion",
            list(DEFAULT_CAMERA_TO_BASE_QUATERNION),
        )
        self.declare_parameter("publish_tf", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._expected_camera_child_frame_id = str(
            self.get_parameter("expected_camera_child_frame_id").value
        )
        self._base_child_frame_id = str(
            self.get_parameter("base_child_frame_id").value
        )
        self._camera_to_base_translation = tuple(
            float(value)
            for value in self.get_parameter("camera_to_base_translation").value
        )
        self._camera_to_base_quaternion = tuple(
            float(value)
            for value in self.get_parameter("camera_to_base_quaternion").value
        )
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        # Validate the configured mount before accepting live odometry.
        transform_camera_odometry_to_base(
            OdometryState(
                position=(0.0, 0.0, 0.0),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_velocity=(0.0, 0.0, 0.0),
                angular_velocity=(0.0, 0.0, 0.0),
            ),
            self._camera_to_base_translation,
            self._camera_to_base_quaternion,
        )

        self._publisher = self.create_publisher(
            Odometry,
            output_topic,
            QoSProfile(depth=10),
        )
        self._subscription = self.create_subscription(
            Odometry,
            input_topic,
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.get_logger().info(
            "odometry adapter active: %s (%s) -> %s (%s), publish_tf=%s"
            % (
                input_topic,
                self._expected_camera_child_frame_id,
                output_topic,
                self._base_child_frame_id,
                self._publish_tf,
            )
        )

    def _on_odometry(self, msg: Odometry) -> None:
        if msg.child_frame_id != self._expected_camera_child_frame_id:
            self.get_logger().error(
                "Ignoring odometry with child_frame_id=%r; expected %r"
                % (msg.child_frame_id, self._expected_camera_child_frame_id)
            )
            return

        camera_state = OdometryState(
            position=(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ),
            orientation_xyzw=(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ),
            linear_velocity=(
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ),
            angular_velocity=(
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ),
        )
        try:
            base_state = transform_camera_odometry_to_base(
                camera_state,
                self._camera_to_base_translation,
                self._camera_to_base_quaternion,
            )
        except ValueError as exc:
            self.get_logger().error(f"Ignoring invalid camera odometry: {exc}")
            return

        output = Odometry()
        output.header = msg.header
        output.child_frame_id = self._base_child_frame_id
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = (
            base_state.position
        )
        (
            output.pose.pose.orientation.x,
            output.pose.pose.orientation.y,
            output.pose.pose.orientation.z,
            output.pose.pose.orientation.w,
        ) = base_state.orientation_xyzw
        (
            output.twist.twist.linear.x,
            output.twist.twist.linear.y,
            output.twist.twist.linear.z,
        ) = base_state.linear_velocity
        (
            output.twist.twist.angular.x,
            output.twist.twist.angular.y,
            output.twist.twist.angular.z,
        ) = base_state.angular_velocity

        # Nav2 consumes the adapted mean pose and twist here. Keep the ZED
        # uncertainty payload intact rather than replacing it with arbitrary
        # confidence values.
        output.pose.covariance = msg.pose.covariance
        output.twist.covariance = msg.twist.covariance
        self._publisher.publish(output)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = output.child_frame_id
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = output.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
