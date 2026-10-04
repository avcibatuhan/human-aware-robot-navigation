"""ROS 2 node: tracked boxes + depth image + intrinsics -> 3D points (camera frame)."""

import copy

import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from human_interfaces.msg import TrackedHumanArray
from human_tracking.depth import sample_depth
from human_tracking.projection import Intrinsics, project_pixel


class HumanLocalizer(Node):
    def __init__(self):
        super().__init__("human_localizer")
        self.declare_parameter("tracks_topic", "/tracked_humans")
        self.declare_parameter("depth_topic", "/human_camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/human_camera/camera_info")
        self.declare_parameter("output_topic", "/tracked_humans_3d")
        self.declare_parameter("depth_window_half_size", 5)
        self.declare_parameter("min_depth", 0.5)
        self.declare_parameter("max_depth", 10.0)
        self.declare_parameter("sync_slop", 0.02)

        self.half_window = self.get_parameter("depth_window_half_size").value
        self.min_depth = self.get_parameter("min_depth").value
        self.max_depth = self.get_parameter("max_depth").value
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            TrackedHumanArray, self.get_parameter("output_topic").value, 10
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        subscribers = [
            message_filters.Subscriber(
                self, TrackedHumanArray, self.get_parameter("tracks_topic").value, qos_profile=qos
            ),
            message_filters.Subscriber(
                self, Image, self.get_parameter("depth_topic").value, qos_profile=qos
            ),
            message_filters.Subscriber(
                self, CameraInfo, self.get_parameter("camera_info_topic").value, qos_profile=qos
            ),
        ]
        # Tracks arrive one detection + tracking cycle after their depth frame,
        # so the queue has to hold a few frames of depth.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            subscribers, queue_size=30, slop=self.get_parameter("sync_slop").value
        )
        self.sync.registerCallback(self.on_frame)

    def on_frame(self, tracks: TrackedHumanArray, depth_msg: Image, info: CameraInfo):
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        intrinsics = Intrinsics.from_k(info.k)

        msg = TrackedHumanArray()
        msg.header.stamp = tracks.header.stamp
        msg.header.frame_id = depth_msg.header.frame_id
        for human in tracks.humans:
            z = sample_depth(
                depth,
                human.bbox_center_x,
                human.bbox_center_y,
                self.half_window,
                self.min_depth,
                self.max_depth,
            )
            if z is None:
                self.get_logger().debug(f"track {human.id}: no valid depth, dropped")
                continue
            located = copy.deepcopy(human)
            x, y, z = project_pixel(human.bbox_center_x, human.bbox_center_y, z, intrinsics)
            located.position.x, located.position.y, located.position.z = x, y, z
            msg.humans.append(located)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = HumanLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
