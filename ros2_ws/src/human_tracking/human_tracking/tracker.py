"""DeepSORT wrapper with plain inputs and outputs. No ROS imports.

The underlying tracker is injected (anything with the ``deep-sort-realtime``
``update_tracks`` interface), so the tests can use a fake.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    """Bounding box in pixels, given by its centre and size."""

    confidence: float
    center_x: float
    center_y: float
    width: float
    height: float


@dataclass(frozen=True)
class TrackedBox:
    track_id: int
    box: Box


def make_deepsort(max_age: int = 30, n_init: int = 3, embedder_gpu: bool = False):
    """Create the real DeepSORT tracker (imported lazily: it loads torch)."""
    from deep_sort_realtime.deepsort_tracker import DeepSort

    return DeepSort(max_age=max_age, n_init=n_init, embedder_gpu=embedder_gpu, half=embedder_gpu)


class HumanTracker:
    def __init__(self, deepsort, publish_coasting: bool = False):
        """``publish_coasting``: also output confirmed tracks that were not matched
        to a detection in the current frame (their box is a Kalman prediction)."""
        self.deepsort = deepsort
        self.publish_coasting = publish_coasting

    def update(self, boxes: list[Box], frame_bgr) -> list[TrackedBox]:
        """Feed one frame's detections; return the confirmed tracks."""
        raw = [
            (
                [b.center_x - b.width / 2, b.center_y - b.height / 2, b.width, b.height],
                b.confidence,
                "person",
            )
            for b in boxes
        ]
        tracked = []
        for track in self.deepsort.update_tracks(raw, frame=frame_bgr):
            if not track.is_confirmed():
                continue
            confidence = track.get_det_conf()
            matched = confidence is not None
            if not matched and not self.publish_coasting:
                continue
            # The matched detection's own box when there is one, else the prediction.
            left, top, right, bottom = track.to_ltrb(orig=matched)
            tracked.append(
                TrackedBox(
                    track_id=int(track.track_id),
                    box=Box(
                        confidence=float(confidence) if matched else 0.0,
                        center_x=float(left + right) / 2,
                        center_y=float(top + bottom) / 2,
                        width=float(right - left),
                        height=float(bottom - top),
                    ),
                )
            )
        return tracked
