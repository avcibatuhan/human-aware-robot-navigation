"""ROS 2 node: person detections + RGB image -> tracked humans with stable IDs."""

import time

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from human_interfaces.msg import TrackedHuman, TrackedHumanArray
from human_tracking.tracker import Box, HumanTracker, make_deepsort


class DeepSortNode(Node):
    def __init__(self):
        super().__init__("deepsort_node")
        self.declare_parameter("max_age", 30)
        self.declare_parameter("n_init", 3)
        self.declare_parameter("embedder_gpu", False)
        self.declare_parameter("torch_threads", 0)
        self.declare_parameter("publish_coasting_tracks", False)
        self.declare_parameter("detections_topic", "/human_detections")
        self.declare_parameter("image_topic", "/human_camera/image_raw")
        self.declare_parameter("tracks_topic", "/tracked_humans")
        self.declare_parameter("sync_slop", 0.02)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "/tracked_humans/debug_image")
        self.declare_parameter("stats_period", 5.0)

        # Several torch processes each grabbing every core slow each other
        # down badly; 0 keeps torch's default.
        if self.get_parameter("torch_threads").value > 0:
            import torch

            torch.set_num_threads(self.get_parameter("torch_threads").value)

        self.tracker = HumanTracker(
            make_deepsort(
                max_age=self.get_parameter("max_age").value,
                n_init=self.get_parameter("n_init").value,
                embedder_gpu=self.get_parameter("embedder_gpu").value,
            ),
            publish_coasting=self.get_parameter("publish_coasting_tracks").value,
        )
        self.bridge = CvBridge()
        self.frame_times = []
        self.stats_period = self.get_parameter("stats_period").value
        self.last_stats = time.monotonic()

        self.publisher = self.create_publisher(
            TrackedHumanArray, self.get_parameter("tracks_topic").value, 10
        )
        self.debug_publisher = None
        if self.get_parameter("publish_debug_image").value:
            self.debug_publisher = self.create_publisher(
                Image, self.get_parameter("debug_image_topic").value, 1
            )

        # A detection message carries the stamp of the image it came from, so
        # the two are matched on (almost) exact time.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        detections = message_filters.Subscriber(
            self, TrackedHumanArray, self.get_parameter("detections_topic").value, qos_profile=qos
        )
        images = message_filters.Subscriber(
            self, Image, self.get_parameter("image_topic").value, qos_profile=qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [detections, images], queue_size=30, slop=self.get_parameter("sync_slop").value
        )
        self.sync.registerCallback(self.on_frame)

    def on_frame(self, detections: TrackedHumanArray, image_msg: Image):
        start = time.perf_counter()
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        boxes = [
            Box(h.confidence, h.bbox_center_x, h.bbox_center_y, h.bbox_width, h.bbox_height)
            for h in detections.humans
        ]
        tracks = self.tracker.update(boxes, image)

        msg = TrackedHumanArray()
        msg.header = detections.header
        for track in tracks:
            human = TrackedHuman()
            human.id = track.track_id
            human.confidence = track.box.confidence
            human.bbox_center_x = track.box.center_x
            human.bbox_center_y = track.box.center_y
            human.bbox_width = track.box.width
            human.bbox_height = track.box.height
            msg.humans.append(human)
        self.publisher.publish(msg)

        if self.debug_publisher is not None:
            self.publish_debug_image(image, tracks, image_msg.header)

        self.frame_times.append((time.perf_counter() - start) * 1000.0)
        now = time.monotonic()
        if now - self.last_stats >= self.stats_period:
            mean_ms = sum(self.frame_times) / len(self.frame_times)
            rate = len(self.frame_times) / (now - self.last_stats)
            self.get_logger().info(
                f"processing time: mean {mean_ms:.1f} ms, max {max(self.frame_times):.1f} ms "
                f"over {len(self.frame_times)} frames ({rate:.1f} Hz)"
            )
            self.frame_times.clear()
            self.last_stats = now

    def publish_debug_image(self, image, tracks, header):
        for track in tracks:
            box = track.box
            x1, y1 = int(box.center_x - box.width / 2), int(box.center_y - box.height / 2)
            x2, y2 = int(box.center_x + box.width / 2), int(box.center_y + box.height / 2)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 140, 255), 2)
            cv2.putText(
                image,
                f"ID {track.track_id}",
                (x1, max(y1 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 140, 255),
                2,
            )
        debug = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug.header = header
        self.debug_publisher.publish(debug)


def main():
    rclpy.init()
    node = DeepSortNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
