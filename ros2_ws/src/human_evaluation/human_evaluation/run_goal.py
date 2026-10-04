"""Send one NavigateToPose goal and report the outcome as JSON.

ros2 run human_evaluation run_goal --x 4.0 --y 0.0 --timeout 180 --output goal.json
ros2 run human_evaluation run_goal --wait-only --start-sim-time 70

``--start-sim-time`` delays the goal until the simulation clock reaches that
value, so paired runs can meet the walking person at the same point of its loop.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_ABORTED: "aborted",
    GoalStatus.STATUS_CANCELED: "canceled",
}


def sim_now(node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


def spin_until(node, condition, timeout: float) -> bool:
    """Spin until ``condition()`` is true; ``timeout`` is wall-clock seconds."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if condition():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return condition()


def run(args) -> dict:
    node = Node(
        "run_goal", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    result = {
        "goal": {"x": args.x, "y": args.y, "yaw": args.yaw, "frame": "map"},
        "timeout": args.timeout,
        "requested_start_sim_time": args.start_sim_time,
        "outcome": "no_action_server",
        "goal_sent_sim_time": None,
        "result_sim_time": None,
        "duration": None,
    }
    if not client.wait_for_server(timeout_sec=60.0):
        return result
    spin_until(node, lambda: sim_now(node) > 0.0, 30.0)
    spin_until(node, lambda: sim_now(node) >= args.start_sim_time, 600.0)

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2)

    # Nav2 rejects goals until it is fully active, so retry for a while.
    handle = None
    for _ in range(30):
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        handle = future.result()
        if handle is not None and handle.accepted:
            break
        handle = None
        spin_until(node, lambda: False, 2.0)
    if handle is None:
        result["outcome"] = "rejected"
        return result

    sent = sim_now(node)
    result["goal_sent_sim_time"] = sent
    result_future = handle.get_result_async()
    finished = spin_until(
        node, lambda: result_future.done() or sim_now(node) - sent > args.timeout, 3600.0
    )
    if finished and result_future.done():
        result["outcome"] = STATUS_NAMES.get(result_future.result().status, "unknown")
    else:
        result["outcome"] = "timeout"
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
    result["result_sim_time"] = sim_now(node)
    result["duration"] = result["result_sim_time"] - sent
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument(
        "--wait-only",
        action="store_true",
        help="only wait until --start-sim-time, send no goal",
    )
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=180.0, help="simulation seconds")
    parser.add_argument("--start-sim-time", type=float, default=0.0)
    parser.add_argument("--output", help="write the JSON result to this file")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    if args.wait_only:
        node = Node(
            "wait_sim_time",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        reached = spin_until(node, lambda: sim_now(node) >= args.start_sim_time, 600.0)
        rclpy.try_shutdown()
        sys.exit(0 if reached else 1)
    result = run(args)
    rclpy.try_shutdown()
    text = json.dumps(result, indent=2)
    if args.output:
        with open(args.output, "w") as f:
            f.write(text + "\n")
    print(text)
    sys.exit(0 if result["outcome"] == "succeeded" else 1)


if __name__ == "__main__":
    main()
