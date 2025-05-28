import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node

from hunavis.utils import goal_from_params


SCENARIO_PARAMS_FILE = "hunavsim.yaml"

DEFAULT_PARAMS_FILE = os.path.join(
    get_package_share_directory("hunavis"),
    "params",
    SCENARIO_PARAMS_FILE,
)


def launch_setup(context, *args, **kwargs):
    use_simulator = LaunchConfiguration("use_simulator")
    params_file = LaunchConfiguration("params_file")
    run_rviz = LaunchConfiguration("run_rviz")

    params_file_val = params_file.perform(context)

    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("hunavis"),
                "rviz",
                "default_sim_view.rviz",
            )
        ],
        output={"both": "log"},
    )

    # Activate the following Nodes only if we are running experiments in the real world
    zed_wrapper_launch = IncludeLaunchDescription(
        # if not `use_simulator`
        condition=IfCondition(NotSubstitution(use_simulator)),
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("hunavis"),
                "/launch/zed_camera.launch.py",
            ]
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "publish_tf": "false",
        }.items(),
    )
    camera_map_tf = Node(
        condition=IfCondition(NotSubstitution(use_simulator)),
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-1.8",
            "--y",
            "1.4",
            "--z",
            "2.6",
            "--yaw",
            f"{math.pi/2}",
            "--pitch",
            "0.28",
            "--roll",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "zed_camera_link",
        ],
    )  # Dummy values; change according to pose changes wrt map

    zed2nav_node = Node(
        condition=IfCondition(NotSubstitution(use_simulator)),
        package="hunavis",
        executable="zed2nav"
    )

    # Load list of human goals from the simulation parameters
    humans_goals_str = goal_from_params(params_file_val)

    people_visualizer_node = Node(
        package="hunavis",
        executable="people_visualizer",
        parameters=[
            {"use_simulator": use_simulator},
            {"goals": humans_goals_str},
        ],
    )
    return [
        zed_wrapper_launch,
        zed2nav_node,
        camera_map_tf,
        people_visualizer_node,
        rviz_node,
    ]


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
                "params_file",
                default_value=DEFAULT_PARAMS_FILE,
                description="Parameter file to use",
            ),
            DeclareLaunchArgument(
                "run_rviz",
                default_value="True",
                description="Whether to use rviz",
                choices=["True", "False"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
