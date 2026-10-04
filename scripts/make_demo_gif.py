#!/usr/bin/env python3
"""Build docs/demo.gif from a demo bag: camera view with track IDs next to the local costmap.

Record the bag with (inside the dev container):

    DEMO=1 scripts/run_experiment.sh social 1 bags/_demo

then run, from the repository root:

    python3 scripts/make_demo_gif.py bags/_demo/social_01/bag docs/demo.gif

The right-hand panel is drawn from /local_costmap/costmap, /odom, /local_plan
and /tracked_humans_map; it is not an RViz screen capture.
"""

import argparse
import bisect
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader

CAMERA_TOPIC = "/tracked_humans/debug_image"
COSTMAP_TOPIC = "/local_costmap/costmap"
PANEL = 360  # px, height of both panels and width of the costmap panel


def stamp(msg_stamp) -> float:
    return msg_stamp.sec + msg_stamp.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def latest(times, items, t):
    """The item with the largest time <= t (or the first one)."""
    index = bisect.bisect_right(times, t) - 1
    return items[max(index, 0)] if items else None


def costmap_colours(grid: np.ndarray) -> np.ndarray:
    """OccupancyGrid values (-1, 0..100) -> BGR, similar to RViz's costmap scheme."""
    value = np.clip(grid, 0, 100).astype(np.float32) / 100.0
    image = np.zeros((*grid.shape, 3), np.uint8)
    image[..., 2] = (value * 255).astype(np.uint8)
    image[..., 1] = (value * 60).astype(np.uint8)
    image[..., 0] = ((1.0 - value) * 90 * (value > 0)).astype(np.uint8)
    image[grid == 0] = (40, 40, 40)
    image[grid == 99] = (200, 0, 200)  # inscribed
    image[grid == 100] = (255, 255, 0)  # lethal
    image[grid < 0] = (70, 70, 70)  # unknown
    return image


def draw_costmap(costmap, robot, humans, plan) -> np.ndarray:
    """Render one costmap message with the robot, tracked humans and local plan (odom frame)."""
    info = costmap.info
    grid = np.asarray(costmap.data, dtype=np.int16).reshape(info.height, info.width)
    scale = PANEL / info.width
    image = cv2.resize(
        np.flipud(costmap_colours(grid)), (PANEL, PANEL), interpolation=cv2.INTER_NEAREST
    )

    def pixel(x, y):
        col = (x - info.origin.position.x) / info.resolution * scale
        row = (info.height - (y - info.origin.position.y) / info.resolution) * scale
        return int(round(col)), int(round(row))

    if plan is not None and len(plan) > 1:
        points = np.array([pixel(x, y) for x, y in plan], dtype=np.int32)
        cv2.polylines(image, [points], False, (255, 160, 60), 2, cv2.LINE_AA)
    if robot is not None:
        x, y, yaw = robot
        centre = pixel(x, y)
        cv2.circle(
            image, centre, int(0.17 / info.resolution * scale), (80, 230, 80), 2, cv2.LINE_AA
        )
        tip = pixel(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw))
        cv2.line(image, centre, tip, (80, 230, 80), 2, cv2.LINE_AA)
    for track_id, x, y in humans:
        centre = pixel(x, y)
        if 0 <= centre[0] < PANEL and 0 <= centre[1] < PANEL:
            cv2.drawMarker(image, centre, (255, 255, 255), cv2.MARKER_CROSS, 12, 2)
            cv2.putText(
                image,
                f"ID {track_id}",
                (centre[0] + 8, centre[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
    return image


def caption(image: np.ndarray, text: str) -> np.ndarray:
    cv2.rectangle(image, (0, 0), (image.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(
        image, text, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA
    )
    return image


def build(bag: Path, output: Path, fps: float, start: float, duration: float, colours: int):
    camera, costmaps, odom, map_to_odom, humans, plans = [], [], [], [], [], []
    topics = {CAMERA_TOPIC, COSTMAP_TOPIC, "/odom", "/tf", "/tracked_humans_map", "/local_plan"}
    with AnyReader([bag]) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        missing = topics - {c.topic for c in connections}
        if missing:
            raise SystemExit(f"bag is missing {sorted(missing)}; record it with DEMO=1")
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            topic = connection.topic
            if topic == CAMERA_TOPIC:
                frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
                camera.append((stamp(msg.header.stamp), frame))
            elif topic == COSTMAP_TOPIC:
                costmaps.append((stamp(msg.header.stamp), msg))
            elif topic == "/odom":
                p = msg.pose.pose
                odom.append(
                    (stamp(msg.header.stamp), (p.position.x, p.position.y, yaw_of(p.orientation)))
                )
            elif topic == "/tf":
                for tf in msg.transforms:
                    if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                        t = tf.transform
                        map_to_odom.append(
                            (
                                stamp(tf.header.stamp),
                                (t.translation.x, t.translation.y, yaw_of(t.rotation)),
                            )
                        )
            elif topic == "/tracked_humans_map":
                people = [(h.id, h.position.x, h.position.y) for h in msg.humans]
                humans.append((stamp(msg.header.stamp), people))
            elif topic == "/local_plan":
                points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                plans.append((stamp(msg.header.stamp), points))

    for series in (camera, costmaps, odom, map_to_odom, humans, plans):
        series.sort(key=lambda item: item[0])

    def split(series):
        return [t for t, _ in series], [item for _, item in series]

    costmap_t, costmap_v = split(costmaps)
    odom_t, odom_v = split(odom)
    tf_t, tf_v = split(map_to_odom)
    human_t, human_v = split(humans)
    plan_t, plan_v = split(plans)

    t0 = camera[0][0] + start
    frames, next_time = [], t0
    for t, frame in camera:
        if t < next_time:
            continue
        if t > t0 + duration:
            break
        next_time += 1.0 / fps

        # Humans are published in the map frame; the local costmap lives in odom.
        people = []
        tf = latest(tf_t, tf_v, t)
        current = latest(human_t, human_v, t) or []
        if tf is not None and human_t and t - latest(human_t, human_t, t) < 0.5:
            ox, oy, oyaw = tf
            for track_id, x, y in current:
                dx, dy = x - ox, y - oy
                people.append(
                    (
                        track_id,
                        math.cos(oyaw) * dx + math.sin(oyaw) * dy,
                        -math.sin(oyaw) * dx + math.cos(oyaw) * dy,
                    )
                )

        left = cv2.resize(frame, (int(PANEL * frame.shape[1] / frame.shape[0]), PANEL))
        left = caption(left.copy(), "Robot camera: YOLOv8 + DeepSORT track IDs")
        costmap = latest(costmap_t, costmap_v, t)
        right = draw_costmap(costmap, latest(odom_t, odom_v, t), people, latest(plan_t, plan_v, t))
        right = caption(right, "Nav2 local costmap with social layer")
        gap = np.full((PANEL, 6, 3), 255, np.uint8)
        combined = np.hstack([left, gap, right])
        frames.append(Image.fromarray(cv2.cvtColor(combined, cv2.COLOR_BGR2RGB)))

    if not frames:
        raise SystemExit("no frames in the selected time range")
    # One shared palette keeps the file small and avoids flicker between frames.
    palette = frames[len(frames) // 2].quantize(colours, method=Image.Quantize.MEDIANCUT)
    quantized = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    output.parent.mkdir(parents=True, exist_ok=True)
    quantized[0].save(
        output,
        save_all=True,
        append_images=quantized[1:],
        duration=int(1000 / fps),
        loop=0,
        optimize=True,
    )
    size_mb = output.stat().st_size / 1e6
    print(
        f"{output}: {len(frames)} frames, {frames[0].size[0]}x{frames[0].size[1]}, {size_mb:.1f} MB"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path, nargs="?", default=Path("docs/demo.gif"))
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--start", type=float, default=0.0, help="seconds into the bag")
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--colours", type=int, default=64)
    args = parser.parse_args()
    build(args.bag, args.output, args.fps, args.start, args.duration, args.colours)


if __name__ == "__main__":
    main()
