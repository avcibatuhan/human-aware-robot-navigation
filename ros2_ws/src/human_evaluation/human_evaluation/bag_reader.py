"""Read the topics the evaluation needs from a rosbag2 (MCAP) directory.

Uses the ``rosbags`` library, so no ROS installation is required. The custom
``human_interfaces`` messages are decoded from the definitions stored in the bag.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

from human_evaluation.human_stats import HumanSample, speed
from human_evaluation.metrics import yaw_from_quaternion
from human_evaluation.validity import HUMAN_TOPICS


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass
class BagData:
    path: Path
    duration: float = 0.0
    topic_counts: dict[str, int] = field(default_factory=dict)
    topic_types: dict[str, str] = field(default_factory=dict)
    # (time, x, y, yaw) arrays
    odom: tuple = ((), (), (), ())
    map_to_odom: tuple = ((), (), (), ())
    cmd_vel: tuple = ((), ())  # (time, linear x)
    # one row per human per message on /tracked_humans_map: time, id, x, y
    humans_map: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))
    # topic -> list of messages, each a list of HumanSample
    human_messages: dict[str, list[list[HumanSample]]] = field(default_factory=dict)

    def topic_rates(self) -> dict[str, float | None]:
        return {
            topic: (count / self.duration if self.duration > 0 else None)
            for topic, count in self.topic_counts.items()
        }


def read_bag(path: Path, map_frame: str = "map", odom_frame: str = "odom") -> BagData:
    data = BagData(path=path)
    odom, map_to_odom, cmd_vel, humans = [], [], [], []
    data.human_messages = {topic: [] for topic in HUMAN_TOPICS}

    with AnyReader([path]) as reader:
        data.duration = reader.duration * 1e-9
        for connection in reader.connections:
            data.topic_counts[connection.topic] = (
                data.topic_counts.get(connection.topic, 0) + connection.msgcount
            )
            data.topic_types[connection.topic] = connection.msgtype

        wanted = {"/odom", "/tf", "/cmd_vel", *HUMAN_TOPICS}
        connections = [c for c in reader.connections if c.topic in wanted]
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            topic = connection.topic
            if topic == "/odom":
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                odom.append(
                    (
                        stamp_seconds(msg.header.stamp),
                        p.x,
                        p.y,
                        yaw_from_quaternion(q.x, q.y, q.z, q.w),
                    )
                )
            elif topic == "/tf":
                for tf in msg.transforms:
                    if tf.header.frame_id == map_frame and tf.child_frame_id == odom_frame:
                        t, q = tf.transform.translation, tf.transform.rotation
                        map_to_odom.append(
                            (
                                stamp_seconds(tf.header.stamp),
                                t.x,
                                t.y,
                                yaw_from_quaternion(q.x, q.y, q.z, q.w),
                            )
                        )
            elif topic == "/cmd_vel":
                # TwistStamped on Jazzy; plain Twist on older setups.
                twist = getattr(msg, "twist", msg)
                stamp = stamp_seconds(msg.header.stamp) if hasattr(msg, "header") else np.nan
                cmd_vel.append((stamp, twist.linear.x))
            else:
                stamp = stamp_seconds(msg.header.stamp)
                data.human_messages[topic].append(
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
                if topic == "/tracked_humans_map":
                    humans.extend((stamp, h.id, h.position.x, h.position.y) for h in msg.humans)

    def columns(rows, width):
        array = np.array(sorted(rows), dtype=float).reshape(-1, width)
        return tuple(array[:, i] for i in range(width))

    data.odom = columns(odom, 4)
    data.map_to_odom = columns(map_to_odom, 4)
    data.cmd_vel = columns(cmd_vel, 2)
    data.humans_map = np.array(sorted(humans), dtype=float).reshape(-1, 4)
    return data
