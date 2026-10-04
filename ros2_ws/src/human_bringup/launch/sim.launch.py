"""Arena world, TurtleBot3 Waffle with RGB-D camera, Gazebo bridges and RViz.

ros2 launch human_bringup sim.launch.py
ros2 launch human_bringup sim.launch.py walking_human:=false gui:=false
"""

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from human_bringup.world_builder import write_world


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def launch_gazebo(context):
    share = Path(get_package_share_directory("human_bringup"))
    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    enabled = [name for name in ("static_human", "walking_human") if is_true(context, name)]
    world = write_world(
        share / "worlds",
        Path(tempfile.gettempdir()) / "human_bringup" / "arena.sdf",
        enabled,
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                "gz_args": f"-r -s -v2 {world}",
                "on_exit_shutdown": "true",
            }.items(),
        )
    ]
    if is_true(context, "gui"):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={"gz_args": "-g -v2", "on_exit_shutdown": "true"}.items(),
            )
        )
    return actions


def launch_rviz(context):
    if not is_true(context, "rviz"):
        return []
    share = Path(get_package_share_directory("human_bringup"))
    fixed_frame = LaunchConfiguration("fixed_frame").perform(context)
    config = Path(tempfile.gettempdir()) / "human_bringup" / "sim.rviz"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(
        (share / "rviz" / "sim.rviz")
        .read_text()
        .replace("Fixed Frame: odom", f"Fixed Frame: {fixed_frame}")
    )
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(config)],
            parameters=[{"use_sim_time": is_true(context, "use_sim_time")}],
            output="log",
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("human_bringup")
    tb3_models = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "models")
    use_sim_time = LaunchConfiguration("use_sim_time")

    with open(os.path.join(share, "urdf", "turtlebot3_waffle_rgbd.urdf")) as f:
        robot_description = f.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="odom",
                description="RViz fixed frame; use map once SLAM or localization is running",
            ),
            DeclareLaunchArgument("static_human", default_value="true"),
            DeclareLaunchArgument("walking_human", default_value="true"),
            DeclareLaunchArgument("x_pose", default_value="-4.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            # TurtleBot3 meshes (model://turtlebot3_common/...) and our models.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", tb3_models),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
            OpaqueFunction(function=launch_gazebo),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name",
                    "waffle",
                    "-file",
                    os.path.join(share, "models", "turtlebot3_waffle_rgbd", "model.sdf"),
                    "-x",
                    LaunchConfiguration("x_pose"),
                    "-y",
                    LaunchConfiguration("y_pose"),
                    "-z",
                    "0.01",
                    "-Y",
                    LaunchConfiguration("yaw"),
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": os.path.join(share, "config", "gz_bridge.yaml"),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
                output="screen",
            ),
            OpaqueFunction(function=launch_rviz),
        ]
    )
