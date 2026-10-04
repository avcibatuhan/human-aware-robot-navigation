"""Perception nodes. Start sim.launch.py first.

ros2 launch human_bringup perception.launch.py
ros2 launch human_bringup perception.launch.py debug_image:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("human_bringup"), "config", "perception.yaml")
    debug = {
        "publish_debug_image": ParameterValue(LaunchConfiguration("debug_image"), value_type=bool)
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug_image",
                default_value="false",
                description="Publish debug images with drawn boxes and track IDs",
            ),
            Node(
                package="human_detection",
                executable="yolo_node",
                parameters=[params, debug],
                output="screen",
            ),
            Node(
                package="human_tracking",
                executable="deepsort_node",
                parameters=[params, debug],
                output="screen",
            ),
            Node(
                package="human_tracking",
                executable="human_localizer",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="human_tracking",
                executable="human_frame_transformer",
                parameters=[params],
                output="screen",
            ),
        ]
    )
