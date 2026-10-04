#!/usr/bin/env bash
# One navigation trial: launch everything, record a bag, send the fixed goal, stop.
#
#   scripts/run_experiment.sh <baseline|social> <trial_id> [output_dir]
#
# Run inside the dev container, from the repository root, with the workspace
# built (docker/run.sh scripts/run_experiment.sh social 1).
#
# Every trial starts a fresh simulation, so the robot and both people are back
# at their start poses. The goal is sent at a fixed simulation time derived from
# the trial id: the baseline and social run of a pair meet the walking person at
# the same point of its loop, and different pairs at different points.
set -o pipefail   # no -u: the ROS setup scripts read unset variables

MODE="${1:?usage: run_experiment.sh <baseline|social> <trial_id> [output_dir]}"
TRIAL="${2:?usage: run_experiment.sh <baseline|social> <trial_id> [output_dir]}"
[[ "$MODE" == "baseline" || "$MODE" == "social" ]] || { echo "mode must be baseline or social" >&2; exit 2; }

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_ROOT="${3:-$REPO_DIR/bags}"
TRIAL_ID="$(printf '%02d' "$((10#$TRIAL))")"
OUT="$OUT_ROOT/${MODE}_${TRIAL_ID}"

GOAL_X=4.0
GOAL_Y=0.0
TIMEOUT=180            # simulation seconds allowed for the goal
EARLIEST_GOAL=75.0     # simulation seconds; startup takes less than this
WALK_PERIOD=17.28      # s, loop time of the walking person (worlds/humans/walking_human.sdf)
SEED="$((10#$TRIAL))"
# Goal time = EARLIEST_GOAL + a seed-derived phase of the walking loop.
START_SIM_TIME="$(python3 -c "
import random
print(round($EARLIEST_GOAL + random.Random($SEED).uniform(0.0, $WALK_PERIOD), 3))")"

TOPICS=(
  /human_camera/image_raw /human_camera/depth/image_raw /human_camera/camera_info
  /human_detections /tracked_humans /tracked_humans_3d /tracked_humans_map
  /tf /tf_static /odom /cmd_vel /plan /local_plan
)
HUMAN_TOPICS=(/human_detections /tracked_humans /tracked_humans_3d /tracked_humans_map)

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/ros2_ws/install/setup.bash"

if [[ -e "$OUT" ]]; then
  echo "$OUT already exists, refusing to overwrite" >&2
  exit 2
fi
mkdir -p "$OUT"
LOG="$OUT/logs"
mkdir -p "$LOG"

PIDS=()
stop_all() {
  for pid in "${PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null
  done
  sleep 3
  for pid in "${PIDS[@]:-}"; do
    [[ -n "$pid" ]] && kill -KILL -- "-$pid" 2>/dev/null
  done
  pkill -KILL -f "gz sim" 2>/dev/null
  pkill -KILL -f "parameter_bridge|component_container|robot_state_publisher" 2>/dev/null
  pkill -KILL -f "yolo_node|deepsort_node|human_localizer|human_frame_transformer" 2>/dev/null
  true
}
trap stop_all EXIT
stop_all   # nothing left over from an earlier trial
PIDS=()

START_WALL="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "[$MODE $TRIAL_ID] launching simulation"
setsid ros2 launch human_bringup sim.launch.py gui:=false rviz:=false >"$LOG/sim.log" 2>&1 &
PIDS+=($!)
until ros2 topic list 2>/dev/null | grep -q /human_camera/image_raw; do sleep 2; done

echo "[$MODE $TRIAL_ID] launching navigation ($MODE) and perception"
setsid ros2 launch human_bringup navigation.launch.py mode:="$MODE" >"$LOG/navigation.log" 2>&1 &
PIDS+=($!)

echo "[$MODE $TRIAL_ID] waiting for the human topics"
READY=true
DEADLINE=$((SECONDS + 150))
for topic in "${HUMAN_TOPICS[@]}"; do
  # echo fails at once while the topic does not exist yet, so retry.
  until timeout 10 ros2 topic echo "$topic" --once >/dev/null 2>&1; do
    if ((SECONDS > DEADLINE)); then
      echo "[$MODE $TRIAL_ID] $topic is not publishing" >&2
      READY=false
      break
    fi
    sleep 1
  done
done

# Record only the run itself: start a few seconds before the goal is sent.
ros2 run human_evaluation run_goal --wait-only \
  --start-sim-time "$(python3 -c "print($START_SIM_TIME - 4.0)")" >/dev/null 2>&1

echo "[$MODE $TRIAL_ID] recording"
setsid ros2 bag record -s mcap --storage-preset-profile zstd_fast -o "$OUT/bag" \
  --use-sim-time "${TOPICS[@]}" >"$LOG/record.log" 2>&1 &
RECORD_PID=$!
sleep 3

echo "[$MODE $TRIAL_ID] goal ($GOAL_X, $GOAL_Y) at simulation time $START_SIM_TIME"
ros2 run human_evaluation run_goal --x "$GOAL_X" --y "$GOAL_Y" --timeout "$TIMEOUT" \
  --start-sim-time "$START_SIM_TIME" --output "$OUT/goal.json" >"$LOG/goal.log" 2>&1
sleep 2

# SIGTERM, not SIGINT: background jobs of a script start with SIGINT ignored.
kill -TERM -- "-$RECORD_PID" 2>/dev/null
timeout 30 tail --pid="$RECORD_PID" -f /dev/null

python3 - "$OUT" "$MODE" "$TRIAL_ID" "$SEED" "$START_WALL" "$READY" \
  "$(git -C "$REPO_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)" <<'PY'
import json, sys
from pathlib import Path

out, mode, trial, seed, start_wall, ready, commit = sys.argv[1:]
goal_file = Path(out) / "goal.json"
goal = json.loads(goal_file.read_text()) if goal_file.exists() else {"outcome": "no_result"}
metadata = {
    "mode": mode,
    "trial_id": trial,
    "seed": int(seed),
    "start_time_utc": start_wall,
    "human_topics_ready": ready == "true",
    "git_commit": commit,
    **goal,
}
(Path(out) / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
print(f"[{mode} {trial}] outcome: {metadata['outcome']}, duration: {metadata.get('duration')}")
PY
