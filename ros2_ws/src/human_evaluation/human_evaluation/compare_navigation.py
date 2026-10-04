"""Navigation metrics per bag and baseline-vs-social deltas per pair.

ros2 run human_evaluation compare_navigation bags/
"""

import argparse
from pathlib import Path

from human_evaluation.analysis import METRICS, analyse_all, pair_results, summarise_pairs


def fmt(value, digits=3):
    return "–" if value is None else f"{value:.{digits}f}"


def print_report(results, pairs, excluded, summary):
    print("Per bag")
    header = (
        f"{'bag':<14}{'valid':>6}{'outcome':>11}{'samples':>9}{'min d':>8}{'p5 d':>8}"
        f"{'mean d':>8}{'path':>8}{'cmd v':>8}{'time':>8}"
    )
    print(header)
    for r in results:
        print(
            f"{r.name:<14}{('yes' if r.valid else 'NO'):>6}{str(r.outcome):>11}{r.human_samples:>9}"
            f"{fmt(r.min_distance):>8}{fmt(r.p5_distance):>8}{fmt(r.mean_distance):>8}"
            f"{fmt(r.path_length):>8}{fmt(r.mean_cmd_vel):>8}{fmt(r.duration, 1):>8}"
        )

    print("\nPer pair (social − baseline)")
    print(f"{'trial':<7}" + "".join(f"{m:>15}" for m in METRICS))
    for pair in pairs:
        print(f"{pair['trial_id']:<7}" + "".join(f"{fmt(pair[f'delta_{m}']):>15}" for m in METRICS))

    print(f"\nSummary over {len(pairs)} pair(s): mean ± sample std")
    for row in summary:
        print(
            f"{row['label']:<38} baseline {fmt(row['baseline_mean'])} ± {fmt(row['baseline_std'])}"
            f"   social {fmt(row['social_mean'])} ± {fmt(row['social_std'])}"
            f"   delta {fmt(row['delta_mean'])} ± {fmt(row['delta_std'])} {row['unit']}"
            f"   (social higher in {row['pairs_social_higher']}/{row['pairs']})"
        )

    print("\nExcluded")
    if not excluded:
        print("none")
    for item in excluded:
        print(f"{item['bag']}: {item['reason']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bags_dir", type=Path, help="directory with <mode>_<trial> folders")
    args = parser.parse_args()

    results = analyse_all(args.bags_dir)
    pairs, excluded = pair_results(results)
    print_report(results, pairs, excluded, summarise_pairs(pairs))


if __name__ == "__main__":
    main()
