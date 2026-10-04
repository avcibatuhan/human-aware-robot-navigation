"""Live check of the human topics: listen for a while, then print a summary.

ros2 run human_evaluation live_summary --ros-args -p duration:=20.0
"""

import time

import rclpy
from rclpy.node import Node

from human_evaluation.human_stats import HumanSample, HumanTopicStats, format_summary, speed
from human_interfaces.msg import TrackedHumanArray

# topic -> (z_min, z_max) that count as plausible for that topic's position.z
TOPICS = {
    "/human_detections": None,  # no 3D position yet
    "/tracked_humans": None,
    "/tracked_humans_3d": (0.5, 10.0),  # depth along the optical axis
    "/tracked_humans_map": (0.0, 2.0),  # height of the sampled point above the floor
}


class LiveSummary(Node):
    def __init__(self):
        super().__init__("live_summary")
        self.declare_parameter("duration", 20.0)
        self.duration = self.get_parameter("duration").value
        self.stats = {}
        for topic, z_band in TOPICS.items():
            low, high = z_band if z_band else (float("-inf"), float("inf"))
            self.stats[topic] = HumanTopicStats(z_min=low, z_max=high)
            self.create_subscription(
                TrackedHumanArray, topic, lambda msg, t=topic: self.on_message(t, msg), 50
            )

    def on_message(self, topic, msg):
        self.stats[topic].add_message(
            [
                HumanSample(
                    track_id=h.id,
                    confidence=h.confidence,
                    z=h.position.z,
                    speed=speed(h.velocity.x, h.velocity.y, h.velocity.z),
                )
                for h in msg.humans
            ]
        )

    def report(self, elapsed):
        print(f"\nHuman topic summary over {elapsed:.1f} s (wall clock)\n")
        for topic, stats in self.stats.items():
            print(format_summary(topic, stats.summary(elapsed)))
            print()


def main():
    rclpy.init()
    node = LiveSummary()
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < node.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.report(time.monotonic() - start)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
