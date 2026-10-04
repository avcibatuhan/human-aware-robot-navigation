#!/usr/bin/env bash
# Drive the robot around the arena while slam.launch.py is running, then save
# the map into human_bringup/maps/. Run inside the dev container:
#
#   ros2 launch human_bringup slam.launch.py      (terminal 1)
#   scripts/build_map.sh                          (terminal 2)
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$REPO_DIR/ros2_ws/src/human_bringup/maps/map"

echo "waiting for slam_toolbox..."
until ros2 topic echo /map --once --field info.resolution >/dev/null 2>&1; do sleep 2; done

# An outer loop past every corner, then an inner pass between the cylinders.
# slam_toolbox only clears cells along beams that hit something within the
# lidar's 3.5 m range, so the middle of the room needs its own pass.
python3 "$REPO_DIR/scripts/drive_waypoints.py" \
  2.5,0 2.5,3 4.3,3 4.3,0 4.3,-3 1.5,-3.2 -4,-3.2 -4,3 -1.2,3.3 1.2,3.3 \
  1.2,1 -2.3,-0.1 -0.7,-1.4 2,-1.4 2,0 0,0

ros2 run nav2_map_server map_saver_cli -f "$OUT" --ros-args -p use_sim_time:=true
ls -la "$OUT".*
