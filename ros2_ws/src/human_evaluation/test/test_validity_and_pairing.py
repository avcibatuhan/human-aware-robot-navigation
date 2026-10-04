from human_evaluation.analysis import BagResult, pair_results, summarise_pairs
from human_evaluation.validity import REQUIRED_TOPICS, is_valid, missing_topics

COMPLETE = dict.fromkeys(REQUIRED_TOPICS, 10)


def run(mode, trial, min_d=1.0, reason=None, **metrics):
    values = {
        "min_distance": min_d,
        "p5_distance": min_d + 0.1,
        "mean_distance": min_d + 1.0,
        "path_length": 8.0,
        "mean_cmd_vel": 0.2,
        "duration": 35.0,
    }
    values.update(metrics)
    return BagResult(
        name=f"{mode}_{trial}",
        mode=mode,
        trial_id=trial,
        outcome="succeeded",
        valid=reason is None,
        missing_topics=[],
        exclusion_reason=reason,
        human_samples=100,
        **values,
    )


def test_bag_with_all_required_topics_is_valid():
    assert is_valid(COMPLETE)
    assert missing_topics(COMPLETE) == []


def test_missing_or_empty_topic_makes_the_bag_invalid():
    counts = dict(COMPLETE)
    del counts["/plan"]
    counts["/tracked_humans_map"] = 0
    assert not is_valid(counts)
    assert missing_topics(counts) == ["/tracked_humans_map", "/plan"]


def test_pairs_and_deltas_are_social_minus_baseline():
    pairs, excluded = pair_results(
        [run("baseline", "01", 0.8), run("social", "01", 1.1, path_length=8.5)]
    )
    assert excluded == []
    (pair,) = pairs
    assert pair["trial_id"] == "01"
    assert round(pair["delta_min_distance"], 6) == 0.3
    assert round(pair["delta_path_length"], 6) == 0.5


def test_excluded_run_drops_its_pair_and_both_are_reported():
    pairs, excluded = pair_results(
        [
            run("baseline", "01"),
            run("social", "01", reason="goal not reached (timeout)"),
            run("baseline", "02"),
            run("social", "02"),
        ]
    )
    assert [p["trial_id"] for p in pairs] == ["02"]
    assert {"bag": "social_01", "reason": "goal not reached (timeout)"} in excluded
    assert {"bag": "baseline_01", "reason": "its pair social_01 was excluded"} in excluded


def test_missing_partner_is_reported():
    pairs, excluded = pair_results([run("baseline", "03")])
    assert pairs == []
    assert excluded == [{"bag": "social_03", "reason": "run missing"}]


def test_summary_mean_std_and_sign_count():
    pairs, _ = pair_results(
        [
            run("baseline", "01", 1.0),
            run("social", "01", 1.2),
            run("baseline", "02", 1.0),
            run("social", "02", 0.9),
            run("baseline", "03", 1.0),
            run("social", "03", 1.5),
        ]
    )
    row = next(r for r in summarise_pairs(pairs) if r["metric"] == "min_distance")
    assert row["pairs"] == 3
    assert round(row["delta_mean"], 6) == 0.2
    assert round(row["delta_std"], 6) == 0.3
    assert row["pairs_social_higher"] == 2
