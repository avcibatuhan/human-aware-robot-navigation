"""ROS 2 node: 3D human positions in the camera frame -> map frame, with velocity."""

import copy

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from human_interfaces.msg import TrackedHumanArray
from human_tracking.filters import JumpFilter, VelocityEstimator
from human_tracking.transform import apply_transform


class HumanFrameTransformer(Node):
    def __init__(self):
        super().__init__("human_frame_transformer")
        self.declare_parameter("input_topic", "/tracked_humans_3d")
        self.declare_parameter("output_topic", "/tracked_humans_map")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("fallback_fixed_frame", "odom")
        self.declare_parameter("max_position_jump", 0.5)
        self.declare_parameter("jump_reset_after", 1.0)
        self.declare_parameter("velocity_smoothing", 0.4)
        self.declare_parameter("velocity_max_gap", 1.0)
        self.declare_parameter("forget_after", 10.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("markers_topic", "/tracked_humans_markers")

        self.target_frame = self.get_parameter("target_frame").value
        self.fixed_frame = self.get_parameter("fallback_fixed_frame").value
        self.tf_timeout = Duration(seconds=self.get_parameter("tf_timeout").value)
        self.forget_after = self.get_parameter("forget_after").value
        self.jump_filter = JumpFilter(
            max_jump=self.get_parameter("max_position_jump").value,
            reset_after=self.get_parameter("jump_reset_after").value,
        )
        self.velocity = VelocityEstimator(
            smoothing=self.get_parameter("velocity_smoothing").value,
            max_gap=self.get_parameter("velocity_max_gap").value,
        )

        # The listener spins in its own thread so the lookup below can wait
        # for the transform without blocking TF reception.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.publisher = self.create_publisher(
            TrackedHumanArray, self.get_parameter("output_topic").value, 10
        )
        self.marker_publisher = None
        if self.get_parameter("publish_markers").value:
            self.marker_publisher = self.create_publisher(
                MarkerArray, self.get_parameter("markers_topic").value, 10
            )
        self.create_subscription(
            TrackedHumanArray, self.get_parameter("input_topic").value, self.on_humans, 10
        )

    def lookup(self, source_frame: str, stamp: Time):
        """Transform from ``source_frame`` at ``stamp`` into the target frame.

        First at the message timestamp. The localization transform (map ->
        odom) is published at a low and irregular rate, so it is often not
        yet available for a fresh camera frame; in that case the camera pose
        is taken at the message timestamp in the odometry frame and combined
        with the latest map -> odom, which changes only slowly.
        """
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp, timeout=self.tf_timeout
            ).transform
        except TransformException as exact_error:
            if not self.fixed_frame:
                self.warn_no_transform(source_frame, exact_error)
                return None
        try:
            return self.tf_buffer.lookup_transform_full(
                self.target_frame, Time(), source_frame, stamp, self.fixed_frame
            ).transform
        except TransformException as error:
            self.warn_no_transform(source_frame, error)
            return None

    def warn_no_transform(self, source_frame, error):
        self.get_logger().warn(
            f"no transform {source_frame} -> {self.target_frame}: {error}",
            throttle_duration_sec=2.0,
        )

    def on_humans(self, msg: TrackedHumanArray):
        tf = self.lookup(msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if tf is None:
            return

        translation = (tf.translation.x, tf.translation.y, tf.translation.z)
        rotation = (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        out = TrackedHumanArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        for human in msg.humans:
            position = apply_transform(
                translation, rotation, (human.position.x, human.position.y, human.position.z)
            )
            if not self.jump_filter.accept(human.id, position, stamp):
                self.get_logger().debug(f"track {human.id}: position jump rejected")
                continue
            velocity = self.velocity.update(human.id, position, stamp)
            mapped = copy.deepcopy(human)
            mapped.position.x, mapped.position.y, mapped.position.z = position
            mapped.velocity.x, mapped.velocity.y, mapped.velocity.z = velocity
            out.humans.append(mapped)

        self.jump_filter.forget_older_than(stamp - self.forget_after)
        self.velocity.forget_older_than(stamp - self.forget_after)
        self.publisher.publish(out)
        if self.marker_publisher is not None:
            self.publish_markers(out)

    def publish_markers(self, humans: TrackedHumanArray):
        markers = MarkerArray()
        for human in humans.humans:
            for kind, namespace, height in (
                (Marker.CYLINDER, "human", 0.0),
                (Marker.TEXT_VIEW_FACING, "human_id", 2.0),
            ):
                marker = Marker()
                marker.header = humans.header
                marker.ns = namespace
                marker.id = human.id
                marker.type = kind
                marker.action = Marker.ADD
                marker.pose.position.x = human.position.x
                marker.pose.position.y = human.position.y
                marker.pose.orientation.w = 1.0
                marker.lifetime = Duration(seconds=1.0).to_msg()
                marker.color.a = 0.9
                if kind == Marker.CYLINDER:
                    marker.pose.position.z = 0.85
                    marker.scale.x = marker.scale.y = 0.4
                    marker.scale.z = 1.7
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.55, 0.0
                else:
                    marker.pose.position.z = height
                    marker.scale.z = 0.35
                    marker.color.r = marker.color.g = marker.color.b = 1.0
                    marker.text = f"ID {human.id}"
                markers.markers.append(marker)
        self.marker_publisher.publish(markers)


def main():
    rclpy.init()
    node = HumanFrameTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
