"""Person detection with a YOLO model. No ROS imports, so it is unit-testable.

The model is injected: anything with the Ultralytics ``predict`` interface
works, which lets the tests use a small fake instead of real weights.
"""

from dataclasses import dataclass

import numpy as np

PERSON_CLASS = 0  # COCO class index


@dataclass(frozen=True)
class Detection:
    """One detected person; the box is in pixels, given by its centre and size."""

    confidence: float
    center_x: float
    center_y: float
    width: float
    height: float

    @property
    def corners(self) -> tuple[int, int, int, int]:
        """Box as integer (x1, y1, x2, y2), for drawing."""
        return (
            int(round(self.center_x - self.width / 2)),
            int(round(self.center_y - self.height / 2)),
            int(round(self.center_x + self.width / 2)),
            int(round(self.center_y + self.height / 2)),
        )


def _to_numpy(values) -> np.ndarray:
    """Accept torch tensors (what Ultralytics returns) or anything array-like."""
    if hasattr(values, "cpu"):
        values = values.cpu().numpy()
    return np.asarray(values, dtype=float)


class PersonDetector:
    def __init__(self, model, confidence_threshold: float = 0.5, device: str = "cpu"):
        self.model = model
        self.confidence_threshold = confidence_threshold
        self.device = device

    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        """Return the people found in a BGR image, most confident first."""
        results = self.model.predict(
            image_bgr,
            conf=self.confidence_threshold,
            classes=[PERSON_CLASS],
            device=self.device,
            verbose=False,
        )
        if not results:
            return []
        boxes = results[0].boxes
        xywh = _to_numpy(boxes.xywh).reshape(-1, 4)
        confidences = _to_numpy(boxes.conf).reshape(-1)
        classes = _to_numpy(boxes.cls).reshape(-1)

        # The model is already asked for persons above the threshold; filter
        # again so the output contract does not depend on the model honouring it.
        detections = [
            Detection(float(conf), float(x), float(y), float(w), float(h))
            for (x, y, w, h), conf, cls in zip(xywh, confidences, classes, strict=True)
            if int(cls) == PERSON_CLASS and conf >= self.confidence_threshold
        ]
        return sorted(detections, key=lambda d: d.confidence, reverse=True)
