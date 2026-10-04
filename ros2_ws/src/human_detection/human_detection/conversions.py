"""Conversion from detections to the human_interfaces messages."""

from human_detection.detector import Detection
from human_interfaces.msg import TrackedHuman, TrackedHumanArray

UNTRACKED_ID = -1  # detections carry no identity until the tracker assigns one


def detections_to_msg(detections: list[Detection], header) -> TrackedHumanArray:
    """Build a TrackedHumanArray with ``id = -1``; position and velocity stay zero."""
    msg = TrackedHumanArray()
    msg.header = header
    for detection in detections:
        human = TrackedHuman()
        human.id = UNTRACKED_ID
        human.confidence = detection.confidence
        human.bbox_center_x = detection.center_x
        human.bbox_center_y = detection.center_y
        human.bbox_width = detection.width
        human.bbox_height = detection.height
        msg.humans.append(human)
    return msg
