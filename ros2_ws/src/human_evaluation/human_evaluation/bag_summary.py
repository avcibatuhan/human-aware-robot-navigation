"""Summarise recorded bags: message counts, rates, human topic statistics, validity.

ros2 run human_evaluation bag_summary bags/social_01 [bags/baseline_01 ...]
"""

import argparse
import sys
from pathlib import Path

from human_evaluation.bag_reader import read_bag
from human_evaluation.human_stats import HumanTopicStats, format_summary
from human_evaluation.validity import HUMAN_TOPICS, REQUIRED_TOPICS, missing_topics

# plausible position.z per topic (see live_summary)
Z_BANDS = {"/tracked_humans_3d": (0.5, 10.0), "/tracked_humans_map": (0.0, 2.0)}


def summarise(trial_dir: Path) -> bool:
    bag_dir = trial_dir / "bag" if (trial_dir / "bag").is_dir() else trial_dir
    print(f"=== {trial_dir} ===")
    try:
        data = read_bag(bag_dir)
    except Exception as error:
        print(f"INVALID: bag unreadable: {error}\n")
        return False

    print(f"duration {data.duration:.1f} s\n")
    print(f"{'topic':<34}{'messages':>10}{'approx. Hz':>12}")
    rates = data.topic_rates()
    for topic in sorted(set(data.topic_counts) | set(REQUIRED_TOPICS)):
        count = data.topic_counts.get(topic, 0)
        rate = rates.get(topic)
        flag = "" if count or topic not in REQUIRED_TOPICS else "   <- required, missing"
        print(f"{topic:<34}{count:>10}{(f'{rate:.1f}' if rate else '–'):>12}{flag}")
    print()

    for topic in HUMAN_TOPICS:
        low, high = Z_BANDS.get(topic, (float("-inf"), float("inf")))
        stats = HumanTopicStats(z_min=low, z_max=high)
        for humans in data.human_messages.get(topic, []):
            stats.add_message(humans)
        print(format_summary(topic, stats.summary(data.duration)))
        print()

    missing = missing_topics(data.topic_counts)
    print("VALID" if not missing else "INVALID, missing: " + ", ".join(missing))
    print()
    return not missing


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bags", nargs="+", type=Path, help="trial or bag directories")
    args = parser.parse_args()
    valid = [summarise(path) for path in args.bags]
    print(f"{sum(valid)} of {len(valid)} bag(s) valid")
    sys.exit(0 if all(valid) else 1)


if __name__ == "__main__":
    main()
