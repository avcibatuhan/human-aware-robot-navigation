import numpy as np
import pytest

pytest.importorskip("human_interfaces", reason="needs the built ROS workspace")

from std_msgs.msg import Header  # noqa: E402
from test_detector import FakeModel  # noqa: E402

from human_detection.conversions import UNTRACKED_ID, detections_to_msg  # noqa: E402
from human_detection.detector import PersonDetector  # noqa: E402


def make_header():
    header = Header()
    header.frame_id = "human_camera_optical_frame"
    header.stamp.sec = 12
    header.stamp.nanosec = 500
    return header


def test_mocked_model_output_becomes_tracked_human_array():
    model = FakeModel(xywh=[[100, 200, 40, 120], [320, 240, 80, 200]], conf=[0.6, 0.9], cls=[0, 0])
    detections = PersonDetector(model).detect(np.zeros((480, 640, 3), dtype=np.uint8))

    msg = detections_to_msg(detections, make_header())

    assert msg.header.frame_id == "human_camera_optical_frame"
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (12, 500)
    assert [h.id for h in msg.humans] == [UNTRACKED_ID, UNTRACKED_ID] == [-1, -1]
    first = msg.humans[0]
    assert first.confidence == pytest.approx(0.9)
    assert (first.bbox_center_x, first.bbox_center_y) == (320.0, 240.0)
    assert (first.bbox_width, first.bbox_height) == (80.0, 200.0)
    # A detection has no 3D information yet.
    assert (first.position.x, first.position.y, first.position.z) == (0.0, 0.0, 0.0)
    assert (first.velocity.x, first.velocity.y, first.velocity.z) == (0.0, 0.0, 0.0)


def test_no_detections_gives_empty_array_with_header():
    msg = detections_to_msg([], make_header())
    assert msg.humans == []
    assert msg.header.frame_id == "human_camera_optical_frame"
