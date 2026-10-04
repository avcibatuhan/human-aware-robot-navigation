"""Summary statistics of a human topic. No ROS imports.

Shared by the live check (``live_summary``) and the offline bag summary: both
feed plain numbers into :class:`HumanTopicStats`.
"""

import math
from dataclasses import dataclass, field

import numpy as np

PERCENTILES = (5, 50, 95)


@dataclass(frozen=True)
class HumanSample:
    """One human in one message, reduced to what the summary needs."""

    track_id: int
    confidence: float
    z: float
    speed: float


@dataclass
class HumanTopicStats:
    """Accumulates the samples of one topic.

    ``z`` is whatever the topic's ``position.z`` means: the depth along the
    optical axis on ``/tracked_humans_3d``, the height in the map on
    ``/tracked_humans_map``. A z outside ``[z_min, z_max]`` or non-finite is
    counted as suspicious.
    """

    z_min: float = 0.5
    z_max: float = 10.0
    messages: int = 0
    empty_messages: int = 0
    samples: list[HumanSample] = field(default_factory=list)

    def add_message(self, humans: list[HumanSample]) -> None:
        self.messages += 1
        if not humans:
            self.empty_messages += 1
        self.samples.extend(humans)

    def summary(self, duration: float | None = None) -> dict:
        ids = sorted({s.track_id for s in self.samples})
        confidences = np.array([s.confidence for s in self.samples], dtype=float)
        z = np.array([s.z for s in self.samples], dtype=float)
        finite_z = z[np.isfinite(z)]
        speeds = np.array([s.speed for s in self.samples], dtype=float)
        suspicious = int(np.sum(~np.isfinite(z) | (z < self.z_min) | (z > self.z_max)))

        def value_range(values):
            return (float(values.min()), float(values.max())) if values.size else None

        return {
            "messages": self.messages,
            "empty_messages": self.empty_messages,
            "rate_hz": self.messages / duration if duration else None,
            "human_samples": len(self.samples),
            "unique_ids": ids,
            "mean_confidence": float(confidences.mean()) if confidences.size else None,
            "z_range": value_range(finite_z),
            "z_percentiles": (
                {p: float(np.percentile(finite_z, p)) for p in PERCENTILES}
                if finite_z.size
                else None
            ),
            "suspicious_z": suspicious,
            "speed_range": value_range(speeds),
        }


def speed(vx: float, vy: float, vz: float = 0.0) -> float:
    return math.sqrt(vx * vx + vy * vy + vz * vz)


def format_summary(topic: str, s: dict) -> str:
    """Human-readable block for one topic."""

    def fmt_range(r):
        return "–" if r is None else f"{r[0]:.2f} … {r[1]:.2f}"

    rate = "–" if s["rate_hz"] is None else f"{s['rate_hz']:.1f} Hz"
    confidence = "–" if s["mean_confidence"] is None else f"{s['mean_confidence']:.3f}"
    percentiles = (
        "–"
        if s["z_percentiles"] is None
        else ", ".join(f"p{p} {v:.2f}" for p, v in s["z_percentiles"].items())
    )
    return "\n".join(
        [
            topic,
            f"  messages         {s['messages']} ({rate}), {s['empty_messages']} without humans",
            f"  human samples    {s['human_samples']}",
            f"  unique IDs       {s['unique_ids']}",
            f"  mean confidence  {confidence}",
            f"  z range          {fmt_range(s['z_range'])}",
            f"  z percentiles    {percentiles}",
            f"  suspicious z     {s['suspicious_z']}",
            f"  speed range      {fmt_range(s['speed_range'])}",
        ]
    )
