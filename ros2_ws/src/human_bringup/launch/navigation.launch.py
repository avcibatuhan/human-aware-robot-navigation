"""Nav2 (AMCL + map server + DWB) and the perception pipeline. Start sim.launch.py first.

ros2 launch human_bringup navigation.launch.py mode:=baseline
ros2 launch human_bringup navigation.launch.py mode:=social

Perception runs in both modes so that baseline runs record the same human
topics; only the social mode feeds them into the local costmap.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory("human_bringup")
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py"
    )
    # mode:=baseline -> nav2_baseline.yaml, mode:=social -> nav2_social.yaml
    params_file = PathJoinSubstitution(
        [share, "config", ["nav2_", LaunchConfiguration("mode"), ".yaml"]]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="social",
                choices=["baseline", "social"],
                description="baseline: plain Nav2; social: Nav2 + social costmap layer",
            ),
            DeclareLaunchArgument("map", default_value=os.path.join(share, "maps", "map.yaml")),
            DeclareLaunchArgument("perception", default_value="true"),
            DeclareLaunchArgument("debug_image", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": params_file,
                    "use_sim_time": "true",
                    "slam": "False",
                    "autostart": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "perception.launch.py")
                ),
                launch_arguments={"debug_image": LaunchConfiguration("debug_image")}.items(),
                condition=IfCondition(LaunchConfiguration("perception")),
            ),
        ]
    )
