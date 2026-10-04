"""Write the navigation comparison to CSV files and bar charts.

ros2 run human_evaluation export_navigation_results bags/ results/
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

from human_evaluation.analysis import (  # noqa: E402
    METRICS,
    analyse_all,
    pair_results,
    summarise_pairs,
)

CHARTS = {
    "min_distance": "delta_min_distance.png",
    "p5_distance": "delta_p5_distance.png",
    "path_length": "delta_path_length.png",
}

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
SECONDARY_INK = "#52514e"
MUTED = "#898781"
GRID = "#e6e5e1"
HIGHER = "#2a78d6"  # social value above baseline
LOWER = "#eb6834"  # social value below baseline


def delta_chart(pairs: pd.DataFrame, metric: str, summary_row: dict, path: Path) -> None:
    """One bar per pair: social minus baseline, around a zero line."""
    label, unit = METRICS[metric]
    deltas = pairs[f"delta_{metric}"]
    trials = pairs["trial_id"].astype(str)

    fig, ax = plt.subplots(figsize=(8, 4.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    bars = ax.bar(
        trials, deltas, width=0.6, color=[HIGHER if d >= 0 else LOWER for d in deltas], zorder=3
    )
    ax.axhline(0, color=SECONDARY_INK, linewidth=1, zorder=4)
    ax.bar_label(bars, labels=[f"{d:+.2f}" for d in deltas], padding=3, fontsize=8, color=INK)

    mean, std = summary_row["delta_mean"], summary_row["delta_std"]
    spread = f" ± {std:.3f}" if std is not None else ""
    ax.set_title(
        f"{label}: social − baseline, per pair",
        loc="left",
        fontsize=12,
        color=INK,
        fontweight="bold",
        pad=24,
    )
    ax.text(
        0,
        1.04,
        f"Mean change {mean:+.3f}{spread} {unit} over {len(pairs)} pairs "
        f"(blue: social higher, orange: social lower)",
        transform=ax.transAxes,
        fontsize=9,
        color=SECONDARY_INK,
    )
    ax.set_xlabel("Trial pair", color=SECONDARY_INK, fontsize=9)
    ax.set_ylabel(f"Change ({unit})", color=SECONDARY_INK, fontsize=9)
    ax.grid(axis="y", color=GRID, linewidth=0.8, zorder=0)
    ax.tick_params(colors=MUTED, labelsize=9, length=0)
    for side in ("top", "right", "left", "bottom"):
        ax.spines[side].set_visible(False)
    ax.margins(y=0.18)
    fig.tight_layout()
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


def export(bags_dir: Path, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    results = analyse_all(bags_dir)
    pairs, excluded = pair_results(results)
    summary = summarise_pairs(pairs)

    pd.DataFrame([r.as_row() for r in results]).to_csv(out_dir / "per_bag.csv", index=False)
    pd.DataFrame(pairs).to_csv(out_dir / "per_pair.csv", index=False)
    pd.DataFrame(summary).to_csv(out_dir / "summary.csv", index=False)
    pd.DataFrame(excluded, columns=["bag", "reason"]).to_csv(out_dir / "excluded.csv", index=False)

    rates = pd.DataFrame(
        [
            {"bag": r.name, "topic": topic, "messages": count, "approx_hz": r.topic_rates[topic]}
            for r in results
            for topic, count in sorted(r.topic_counts.items())
        ]
    )
    rates.to_csv(out_dir / "topic_rates.csv", index=False)

    if pairs:
        frame = pd.DataFrame(pairs)
        by_metric = {row["metric"]: row for row in summary}
        for metric, filename in CHARTS.items():
            delta_chart(frame, metric, by_metric[metric], out_dir / filename)

    print(
        f"{len(results)} bag(s), {len(pairs)} usable pair(s), {len(excluded)} exclusion(s) "
        f"-> {out_dir}"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bags_dir", type=Path)
    parser.add_argument("out_dir", type=Path, nargs="?", default=Path("results"))
    args = parser.parse_args()
    export(args.bags_dir, args.out_dir)


if __name__ == "__main__":
    main()
