import math

import pytest

from human_evaluation.human_stats import HumanSample, HumanTopicStats, format_summary, speed


def test_empty_topic():
    summary = HumanTopicStats().summary(duration=10.0)
    assert summary["messages"] == 0
    assert summary["rate_hz"] == 0.0
    assert summary["unique_ids"] == []
    assert summary["mean_confidence"] is None
    assert summary["z_range"] is None
    assert summary["z_percentiles"] is None
    assert summary["speed_range"] is None
    assert "–" in format_summary("/topic", summary)


def test_counts_ids_confidence_and_rate():
    stats = HumanTopicStats()
    stats.add_message([HumanSample(1, 0.9, 2.0, 0.0), HumanSample(2, 0.7, 4.0, 0.5)])
    stats.add_message([HumanSample(1, 0.8, 2.2, 0.1)])
    stats.add_message([])
    summary = stats.summary(duration=1.5)
    assert summary["messages"] == 3
    assert summary["empty_messages"] == 1
    assert summary["rate_hz"] == pytest.approx(2.0)
    assert summary["human_samples"] == 3
    assert summary["unique_ids"] == [1, 2]
    assert summary["mean_confidence"] == pytest.approx(0.8)
    assert summary["z_range"] == pytest.approx((2.0, 4.0))
    assert summary["speed_range"] == pytest.approx((0.0, 0.5))


def test_percentiles():
    stats = HumanTopicStats()
    stats.add_message([HumanSample(1, 0.9, float(z), 0.0) for z in range(1, 11)])
    percentiles = stats.summary()["z_percentiles"]
    assert percentiles[50] == pytest.approx(5.5)
    assert percentiles[5] == pytest.approx(1.45)
    assert percentiles[95] == pytest.approx(9.55)


def test_suspicious_z_counts_out_of_band_and_non_finite():
    stats = HumanTopicStats(z_min=0.5, z_max=10.0)
    stats.add_message(
        [
            HumanSample(1, 0.9, 0.2, 0.0),
            HumanSample(1, 0.9, 3.0, 0.0),
            HumanSample(1, 0.9, 11.0, 0.0),
            HumanSample(1, 0.9, math.nan, 0.0),
            HumanSample(1, 0.9, math.inf, 0.0),
        ]
    )
    summary = stats.summary()
    assert summary["suspicious_z"] == 4
    assert summary["z_range"] == pytest.approx((0.2, 11.0))  # non-finite values excluded


def test_speed():
    assert speed(3.0, 4.0) == pytest.approx(5.0)
    assert speed(1.0, 2.0, 2.0) == pytest.approx(3.0)
