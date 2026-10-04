import pytest

from human_tracking.tracker import Box, HumanTracker, TrackedBox


class FakeTrack:
    def __init__(self, track_id, confirmed, det_conf, ltrb, orig_ltrb=None):
        self.track_id = str(track_id)
        self._confirmed = confirmed
        self._det_conf = det_conf
        self._ltrb = ltrb
        self._orig_ltrb = orig_ltrb or ltrb

    def is_confirmed(self):
        return self._confirmed

    def get_det_conf(self):
        return self._det_conf

    def to_ltrb(self, orig=False):
        return self._orig_ltrb if orig else self._ltrb


class FakeDeepSort:
    def __init__(self, tracks):
        self.tracks = tracks
        self.calls = []

    def update_tracks(self, raw_detections, frame=None):
        self.calls.append((raw_detections, frame))
        return self.tracks


def test_detections_are_passed_as_left_top_width_height():
    deepsort = FakeDeepSort([])
    HumanTracker(deepsort).update([Box(0.9, 320.0, 240.0, 80.0, 200.0)], frame_bgr="frame")
    assert deepsort.calls == [([([280.0, 140.0, 80.0, 200.0], 0.9, "person")], "frame")]


def test_only_confirmed_tracks_are_returned():
    deepsort = FakeDeepSort(
        [
            FakeTrack(1, confirmed=True, det_conf=0.9, ltrb=(280, 140, 360, 340)),
            FakeTrack(2, confirmed=False, det_conf=0.8, ltrb=(0, 0, 10, 10)),
        ]
    )
    assert HumanTracker(deepsort).update([], None) == [
        TrackedBox(1, Box(pytest.approx(0.9), 320.0, 240.0, 80.0, 200.0))
    ]


def test_matched_track_uses_the_detection_box_not_the_prediction():
    track = FakeTrack(
        7, confirmed=True, det_conf=0.8, ltrb=(0, 0, 10, 10), orig_ltrb=(100, 100, 140, 220)
    )
    (tracked,) = HumanTracker(FakeDeepSort([track])).update([], None)
    assert tracked == TrackedBox(7, Box(pytest.approx(0.8), 120.0, 160.0, 40.0, 120.0))


def test_coasting_tracks_are_dropped_by_default():
    track = FakeTrack(3, confirmed=True, det_conf=None, ltrb=(0, 0, 10, 20))
    assert HumanTracker(FakeDeepSort([track])).update([], None) == []


def test_coasting_tracks_can_be_published_with_zero_confidence():
    track = FakeTrack(3, confirmed=True, det_conf=None, ltrb=(0, 0, 10, 20))
    tracker = HumanTracker(FakeDeepSort([track]), publish_coasting=True)
    assert tracker.update([], None) == [TrackedBox(3, Box(0.0, 5.0, 10.0, 10.0, 20.0))]
