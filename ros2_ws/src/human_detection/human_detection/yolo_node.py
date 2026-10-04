"""ROS 2 node: RGB image -> person detections (TrackedHumanArray, id = -1)."""

import os
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from human_detection.conversions import detections_to_msg
from human_detection.detector import PersonDetector
from human_interfaces.msg import TrackedHumanArray


def resolve_weights(weights: str, weights_dir: str) -> str:
    """Place bare weight names in ``weights_dir`` so the download is cached there."""
    path = Path(weights).expanduser()
    if path.is_absolute() or path.parent != Path("."):
        return str(path)
    directory = Path(weights_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    return str(directory / path.name)


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.declare_parameter("weights", "yolov8n.pt")
        self.declare_parameter("weights_dir", "~/.cache/human_detection")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("image_topic", "/human_camera/image_raw")
        self.declare_parameter("detections_topic", "/human_detections")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "/human_detections/debug_image")
        self.declare_parameter("stats_period", 5.0)

        weights = resolve_weights(
            self.get_parameter("weights").value, self.get_parameter("weights_dir").value
        )
        # Imported here so the module can be imported without ultralytics.
        os.environ.setdefault("YOLO_CONFIG_DIR", str(Path(weights).parent))
        from ultralytics import YOLO

        self.get_logger().info(f"loading {weights} (downloaded on first run)")
        self.detector = PersonDetector(
            YOLO(weights),
            confidence_threshold=self.get_parameter("confidence_threshold").value,
            device=self.get_parameter("device").value,
        )
        self.bridge = CvBridge()
        self.frame_times = []
        self.stats_period = self.get_parameter("stats_period").value
        self.last_stats = time.monotonic()

        self.publisher = self.create_publisher(
            TrackedHumanArray, self.get_parameter("detections_topic").value, 10
        )
        self.debug_publisher = None
        if self.get_parameter("publish_debug_image").value:
            self.debug_publisher = self.create_publisher(
                Image, self.get_parameter("debug_image_topic").value, 1
            )

        # Reliable: with best effort about half of the 900 kB frames are lost.
        # Depth 1: when inference is slower than the camera, process the
        # newest frame instead of queueing stale ones.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1
        )
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos)

    def on_image(self, msg: Image):
        start = time.perf_counter()
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = self.detector.detect(image)
        self.publisher.publish(detections_to_msg(detections, msg.header))
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        if self.debug_publisher is not None:
            self.publish_debug_image(image, detections, msg.header)

        self.get_logger().debug(f"frame: {len(detections)} person(s), {elapsed_ms:.1f} ms")
        self.frame_times.append(elapsed_ms)
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

    def publish_debug_image(self, image, detections, header):
        for detection in detections:
            x1, y1, x2, y2 = detection.corners
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(
                image,
                f"person {detection.confidence:.2f}",
                (x1, max(y1 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 200, 0),
                2,
            )
        debug = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug.header = header
        self.debug_publisher.publish(debug)


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
