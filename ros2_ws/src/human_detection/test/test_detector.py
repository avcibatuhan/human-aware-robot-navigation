from types import SimpleNamespace

import numpy as np
import pytest

from human_detection.detector import PERSON_CLASS, Detection, PersonDetector

IMAGE = np.zeros((480, 640, 3), dtype=np.uint8)


class FakeTensor:
    """Mimics the torch tensors Ultralytics returns (``.cpu().numpy()``)."""

    def __init__(self, values):
        self.values = np.asarray(values, dtype=np.float32)

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class FakeModel:
    def __init__(self, xywh, conf, cls, tensors=False):
        wrap = FakeTensor if tensors else np.asarray
        self.boxes = SimpleNamespace(xywh=wrap(xywh), conf=wrap(conf), cls=wrap(cls))
        self.calls = []

    def predict(self, image, **kwargs):
        self.calls.append(kwargs)
        return [SimpleNamespace(boxes=self.boxes)]


def test_model_is_asked_for_persons_above_threshold_on_cpu():
    model = FakeModel(np.empty((0, 4)), [], [])
    PersonDetector(model, confidence_threshold=0.5).detect(IMAGE)
    assert model.calls == [
        {"conf": 0.5, "classes": [PERSON_CLASS], "device": "cpu", "verbose": False}
    ]


def test_no_boxes_gives_no_detections():
    assert PersonDetector(FakeModel(np.empty((0, 4)), [], [])).detect(IMAGE) == []


def test_empty_result_list_gives_no_detections():
    model = SimpleNamespace(predict=lambda image, **kwargs: [])
    assert PersonDetector(model).detect(IMAGE) == []


@pytest.mark.parametrize("tensors", [False, True])
def test_boxes_become_detections_sorted_by_confidence(tensors):
    model = FakeModel(
        xywh=[[100, 200, 40, 120], [320, 240, 80, 200]],
        conf=[0.6, 0.9],
        cls=[0, 0],
        tensors=tensors,
    )
    detections = PersonDetector(model).detect(IMAGE)
    assert detections == [
        Detection(pytest.approx(0.9), 320.0, 240.0, 80.0, 200.0),
        Detection(pytest.approx(0.6), 100.0, 200.0, 40.0, 120.0),
    ]


def test_other_classes_and_low_confidence_are_dropped():
    model = FakeModel(
        xywh=[[100, 200, 40, 120], [320, 240, 80, 200], [50, 50, 10, 10]],
        conf=[0.95, 0.4, 0.8],
        cls=[16, 0, 0],  # a dog, a weak person, a person
    )
    detections = PersonDetector(model, confidence_threshold=0.5).detect(IMAGE)
    assert [d.confidence for d in detections] == [pytest.approx(0.8)]


def test_corners():
    assert Detection(0.9, 320.0, 240.0, 80.0, 200.0).corners == (280, 140, 360, 340)
