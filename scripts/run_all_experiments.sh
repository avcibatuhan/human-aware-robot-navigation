#!/usr/bin/env bash
# Run N baseline/social pairs back to back (inside the dev container).
#
#   scripts/run_all_experiments.sh [pairs=10] [output_dir=bags]
#
# Trials that already have an output folder are skipped, so an interrupted
# batch can be resumed.
set -o pipefail

PAIRS="${1:-10}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_ROOT="${2:-$REPO_DIR/bags}"

for trial in $(seq 1 "$PAIRS"); do
  for mode in baseline social; do
    if [[ -e "$OUT_ROOT/${mode}_$(printf '%02d' "$trial")" ]]; then
      echo "[$mode $trial] already recorded, skipping"
      continue
    fi
    "$REPO_DIR/scripts/run_experiment.sh" "$mode" "$trial" "$OUT_ROOT"
  done
done
