#!/usr/bin/env python3
"""Drive the robot through a list of waypoints using odometry only.

Used for the mapping run, where Nav2 cannot plan yet because the map does not
exist. Waypoints are in the odom frame, which equals the world frame when the
robot is spawned at the origin.

    scripts/drive_waypoints.py 2.5,0 2.5,3 4.3,3
"""

import math
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

LINEAR_SPEED = 0.25  # m/s
ANGULAR_GAIN = 1.5
MAX_ANGULAR = 0.8  # rad/s
TURN_IN_PLACE = 0.5  # rad: above this heading error, stop and rotate
REACHED = 0.15  # m


class WaypointDriver(Node):
    def __init__(self, waypoints):
        super().__init__(
            "waypoint_driver",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.waypoints = list(waypoints)
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

    def on_odom(self, msg):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_footprint"
        if self.waypoints:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            gx, gy = self.waypoints[0]
            dx, dy = gx - p.x, gy - p.y
            if math.hypot(dx, dy) < REACHED:
                self.get_logger().info(f"reached ({gx}, {gy})")
                self.waypoints.pop(0)
            else:
                error = math.atan2(
                    math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw)
                )
                cmd.twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, ANGULAR_GAIN * error))
                if abs(error) < TURN_IN_PLACE:
                    cmd.twist.linear.x = LINEAR_SPEED
        self.pub.publish(cmd)


def main():
    waypoints = [tuple(float(v) for v in arg.split(",")) for arg in sys.argv[1:]]
    if not waypoints:
        sys.exit(__doc__)
    rclpy.init()
    node = WaypointDriver(waypoints)
    while rclpy.ok() and node.waypoints:
        rclpy.spin_once(node, timeout_sec=0.5)
    for _ in range(5):  # make sure the final stop command is delivered
        rclpy.spin_once(node, timeout_sec=0.2)


if __name__ == "__main__":
    main()
