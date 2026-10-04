"""Mapping run: arena without humans, robot at the origin, and slam_toolbox.

The robot starts at the world origin so the map frame coincides with the world
frame. Drive it with scripts/build_map.sh, which also saves the map.

ros2 launch human_bringup slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("human_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "sim.launch.py")),
                launch_arguments={
                    "static_human": "false",
                    "walking_human": "false",
                    "x_pose": "0.0",
                    "y_pose": "0.0",
                    "yaw": "0.0",
                    "fixed_frame": "map",
                    "gui": LaunchConfiguration("gui"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("slam_toolbox"),
                        "launch",
                        "online_async_launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "slam_params_file": os.path.join(share, "config", "slam_toolbox.yaml"),
                }.items(),
            ),
        ]
    )
