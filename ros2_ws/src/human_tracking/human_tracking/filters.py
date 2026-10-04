"""Per-track position-jump rejection and velocity estimation. No ROS imports."""

import math
from dataclasses import dataclass

Point = tuple[float, float, float]


def distance(a: Point, b: Point) -> float:
    return math.dist(a, b)


@dataclass
class _Accepted:
    position: Point
    time: float


class JumpFilter:
    """Reject a track's new position if it is implausibly far from the last accepted one.

    A track that keeps being rejected would be stuck at a stale position
    forever, so once the last accepted position is older than ``reset_after``
    seconds the next measurement is accepted unconditionally.
    """

    def __init__(self, max_jump: float = 1.0, reset_after: float = 1.0):
        self.max_jump = max_jump
        self.reset_after = reset_after
        self._last: dict[int, _Accepted] = {}

    def accept(self, track_id: int, position: Point, time: float) -> bool:
        last = self._last.get(track_id)
        if (
            last is not None
            and time - last.time <= self.reset_after
            and distance(position, last.position) > self.max_jump
        ):
            return False
        self._last[track_id] = _Accepted(position, time)
        return True

    def forget_older_than(self, time: float) -> None:
        self._last = {k: v for k, v in self._last.items() if v.time >= time}


class VelocityEstimator:
    """Finite difference of accepted positions per track, exponentially smoothed.

    ``smoothing`` is the weight of the newest finite difference (1.0 = no
    smoothing). A track's first sample has zero velocity. A gap longer than
    ``max_gap`` restarts the estimate, because a difference over a long gap
    says little about the current motion.
    """

    def __init__(self, smoothing: float = 0.4, max_gap: float = 1.0):
        if not 0.0 < smoothing <= 1.0:
            raise ValueError("smoothing must be in (0, 1]")
        self.smoothing = smoothing
        self.max_gap = max_gap
        self._last: dict[int, _Accepted] = {}
        self._velocity: dict[int, Point] = {}

    def update(self, track_id: int, position: Point, time: float) -> Point:
        last = self._last.get(track_id)
        dt = None if last is None else time - last.time
        if dt is not None and dt <= 0.0:
            # Duplicate or out-of-order sample: keep the current estimate.
            return self._velocity.get(track_id, (0.0, 0.0, 0.0))

        self._last[track_id] = _Accepted(position, time)
        if dt is None or dt > self.max_gap:
            self._velocity[track_id] = (0.0, 0.0, 0.0)
            return self._velocity[track_id]

        raw = tuple((p - q) / dt for p, q in zip(position, last.position, strict=True))
        previous = self._velocity.get(track_id)
        if previous is None or previous == (0.0, 0.0, 0.0):
            smoothed = raw
        else:
            a = self.smoothing
            smoothed = tuple(a * r + (1.0 - a) * p for r, p in zip(raw, previous, strict=True))
        self._velocity[track_id] = smoothed
        return smoothed

    def forget_older_than(self, time: float) -> None:
        stale = [k for k, v in self._last.items() if v.time < time]
        for key in stale:
            self._last.pop(key, None)
            self._velocity.pop(key, None)
