"""Per-bag navigation metrics and baseline-vs-social pairing."""

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from human_evaluation.bag_reader import BagData, read_bag
from human_evaluation.metrics import (
    distance_stats,
    mean_and_std,
    mean_or_none,
    path_length,
    robot_positions_in_map,
)
from human_evaluation.validity import missing_topics

# metric name -> (label, unit)
METRICS = {
    "min_distance": ("Minimum human–robot distance", "m"),
    "p5_distance": ("5th percentile human–robot distance", "m"),
    "mean_distance": ("Average human–robot distance", "m"),
    "path_length": ("Path length (odometry)", "m"),
    "mean_cmd_vel": ("Average commanded linear velocity", "m/s"),
    "duration": ("Time to goal", "s"),
}


@dataclass
class BagResult:
    name: str
    mode: str | None
    trial_id: str | None
    outcome: str | None
    valid: bool
    missing_topics: list[str]
    exclusion_reason: str | None
    human_samples: int = 0
    min_distance: float | None = None
    p5_distance: float | None = None
    mean_distance: float | None = None
    path_length: float | None = None
    mean_cmd_vel: float | None = None
    duration: float | None = None
    topic_counts: dict[str, int] = field(default_factory=dict)
    topic_rates: dict[str, float | None] = field(default_factory=dict)

    def as_row(self) -> dict:
        row = asdict(self)
        row.pop("topic_counts")
        row.pop("topic_rates")
        row["missing_topics"] = " ".join(self.missing_topics)
        return row


def within(times, window):
    times = np.asarray(times, dtype=float)
    if window is None:
        return np.ones(times.shape, dtype=bool)
    return (times >= window[0]) & (times <= window[1])


def compute_metrics(data: BagData, window=None) -> dict:
    """The section 3.7 metrics, restricted to ``window = (start, end)`` if given."""
    metrics = {}

    humans = data.humans_map[within(data.humans_map[:, 0], window)]
    if humans.shape[0] and len(data.odom[0]) and len(data.map_to_odom[0]):
        robot_x, robot_y = robot_positions_in_map(data.odom, data.map_to_odom, humans[:, 0])
        distances = np.hypot(humans[:, 2] - robot_x, humans[:, 3] - robot_y)
    else:
        distances = np.empty(0)
    stats = distance_stats(distances)
    metrics["human_samples"] = stats.pop("samples")
    metrics.update(stats)

    odom_mask = within(data.odom[0], window)
    metrics["path_length"] = path_length(data.odom[1][odom_mask], data.odom[2][odom_mask])
    metrics["mean_cmd_vel"] = mean_or_none(data.cmd_vel[1][within(data.cmd_vel[0], window)])
    return metrics


def load_metadata(trial_dir: Path) -> dict:
    file = trial_dir / "metadata.json"
    return json.loads(file.read_text()) if file.exists() else {}


def analyse_trial(trial_dir: Path) -> BagResult:
    """Analyse one trial directory written by scripts/run_experiment.sh."""
    metadata = load_metadata(trial_dir)
    mode, trial_id = metadata.get("mode"), metadata.get("trial_id")
    if mode is None and "_" in trial_dir.name:
        mode, trial_id = trial_dir.name.rsplit("_", 1)
    outcome = metadata.get("outcome")
    result = BagResult(
        name=trial_dir.name,
        mode=mode,
        trial_id=trial_id,
        outcome=outcome,
        valid=False,
        missing_topics=[],
        exclusion_reason=None,
    )

    bag_dir = trial_dir / "bag"
    if not (bag_dir / "metadata.yaml").exists():
        result.exclusion_reason = "no bag recorded"
        return result
    try:
        data = read_bag(bag_dir)
    except Exception as error:  # a truncated or corrupt bag must be reported, not crash the run
        result.exclusion_reason = f"bag unreadable: {error}"
        return result

    result.topic_counts = data.topic_counts
    result.topic_rates = data.topic_rates()
    result.missing_topics = missing_topics(data.topic_counts)
    result.valid = not result.missing_topics

    sent, done = metadata.get("goal_sent_sim_time"), metadata.get("result_sim_time")
    window = (sent, done) if sent is not None and done is not None else None
    for key, value in compute_metrics(data, window).items():
        setattr(result, key, value)
    result.duration = metadata.get("duration")

    if not result.valid:
        result.exclusion_reason = "invalid bag, missing: " + ", ".join(result.missing_topics)
    elif outcome != "succeeded":
        result.exclusion_reason = f"goal not reached ({outcome})"
    elif result.human_samples == 0:
        result.exclusion_reason = "no human samples during the run"
    return result


def analyse_all(bags_dir: Path) -> list[BagResult]:
    trials = sorted(p for p in bags_dir.iterdir() if p.is_dir() and not p.name.startswith("_"))
    return [analyse_trial(trial) for trial in trials]


def pair_results(results: list[BagResult]) -> tuple[list[dict], list[dict]]:
    """Pair baseline and social runs by trial id.

    Returns ``(pairs, excluded)``. A pair is used only if both runs are valid,
    reached the goal and saw humans; everything else is listed in ``excluded``
    with the reason.
    """
    by_trial: dict[str, dict[str, BagResult]] = {}
    for result in results:
        if result.mode in ("baseline", "social") and result.trial_id is not None:
            by_trial.setdefault(result.trial_id, {})[result.mode] = result

    excluded = [
        {"bag": r.name, "reason": r.exclusion_reason} for r in results if r.exclusion_reason
    ]
    pairs = []
    for trial_id in sorted(by_trial):
        baseline, social = by_trial[trial_id].get("baseline"), by_trial[trial_id].get("social")
        for mode, run in (("baseline", baseline), ("social", social)):
            if run is None:
                excluded.append({"bag": f"{mode}_{trial_id}", "reason": "run missing"})
        if baseline is None or social is None:
            continue
        if baseline.exclusion_reason or social.exclusion_reason:
            # The usable half of a broken pair is excluded too; say so.
            for run, other in ((baseline, social), (social, baseline)):
                if not run.exclusion_reason:
                    excluded.append(
                        {"bag": run.name, "reason": f"its pair {other.name} was excluded"}
                    )
            continue
        pair = {"trial_id": trial_id}
        for metric in METRICS:
            b, s = getattr(baseline, metric), getattr(social, metric)
            pair[f"baseline_{metric}"] = b
            pair[f"social_{metric}"] = s
            pair[f"delta_{metric}"] = None if b is None or s is None else s - b
        pairs.append(pair)
    return pairs, excluded


def summarise_pairs(pairs: list[dict]) -> list[dict]:
    """Mean and standard deviation over the pairs, per metric."""
    summary = []
    for metric, (label, unit) in METRICS.items():
        row = {"metric": metric, "label": label, "unit": unit, "pairs": len(pairs)}
        for prefix in ("baseline", "social", "delta"):
            values = [p[f"{prefix}_{metric}"] for p in pairs if p[f"{prefix}_{metric}"] is not None]
            row[f"{prefix}_mean"], row[f"{prefix}_std"] = mean_and_std(values)
        deltas = [p[f"delta_{metric}"] for p in pairs if p[f"delta_{metric}"] is not None]
        row["pairs_social_higher"] = sum(d > 0 for d in deltas)
        summary.append(row)
    return summary
