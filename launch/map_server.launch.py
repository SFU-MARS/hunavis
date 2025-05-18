import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP_PATH = os.path.join(
    get_package_share_directory("hunavis"), "maps", "empty_room.yaml"
)


def launch_setup(context, *args, **kwargs):
    use_simulator = LaunchConfiguration("use_simulator")
    map_path = LaunchConfiguration("map_path")

    use_sim_time = LaunchConfiguration("use_simulator").perform(context) == "True"
    map_path_val = LaunchConfiguration("map_path").perform(context)

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_path_val}],
    )
    lifecycle_nodes = ["map_server"]

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
        ],
    )
    return [map_server_cmd, start_lifecycle_manager_cmd]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulator",
                default_value="True",
                description="Whether to use simulator",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "map_path",
                default_value=DEFAULT_MAP_PATH,
                description="Map path to use",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
