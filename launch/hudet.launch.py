import os

import yaml
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
CAMERA_TF_PARAMS_FILE = "tf_keyboard_publisher.yaml"

DEFAULT_PARAMS_FILES = {
    "scenario": os.path.join(
        get_package_share_directory("hunavis"),
        "params",
        SCENARIO_PARAMS_FILE,
    ),
    "camera_tf": os.path.join(
        get_package_share_directory("hunavis"),
        "params",
        CAMERA_TF_PARAMS_FILE,
    ),
    "zed_launch": os.path.join(
        get_package_share_directory("hunavis"), "params", "zed_launch_args.yaml"
    ),
}


def launch_setup(context, *args, **kwargs):
    use_simulator = LaunchConfiguration("use_simulator")
    scenario_params_file = LaunchConfiguration("scenario_params_file")
    camera_tf_params_file = LaunchConfiguration("camera_tf_params_file")
    zed_launch_args_file = LaunchConfiguration("zed_launch_args_file")
    run_rviz = LaunchConfiguration("run_rviz")

    scenario_params_file_val = scenario_params_file.perform(context)
    zed_launch_args_file_val = zed_launch_args_file.perform(context)

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
    with open(zed_launch_args_file_val, "r") as f:
        zed_args_dict = yaml.safe_load(f)

    zed_launch_args = [(key, str(value)) for key, value in zed_args_dict.items()]

    zed_wrapper_launch = IncludeLaunchDescription(
        condition=IfCondition(NotSubstitution(use_simulator)),  # if not use_simulator
        launch_description_source=PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("zed_wrapper"),
                    "launch",
                    "zed_camera.launch.py",
                )
            ]
        ),
        launch_arguments=zed_launch_args,
    )
    camera_map_tf = Node(
        condition=IfCondition(NotSubstitution(use_simulator)),
        package="hunavis",
        executable="tf_keyboard_publisher",
        parameters=[
            camera_tf_params_file,
        ],
    )

    # Load list of human goals from the simulation parameters
    humans_goals_str = goal_from_params(scenario_params_file_val)

    people_visualizer_node = Node(
        package="hunavis",
        executable="people_visualizer",
        parameters=[
            {"use_simulator": use_simulator},
            {"goals": humans_goals_str},
        ],
    )
    return [
        rviz_node,
        zed_wrapper_launch,
        camera_map_tf,
        people_visualizer_node,
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
                "scenario_params_file",
                default_value=DEFAULT_PARAMS_FILES["scenario"],
                description="Parameter file to use for scenario",
            ),
            DeclareLaunchArgument(
                "camera_tf_params_file",
                default_value=DEFAULT_PARAMS_FILES["camera_tf"],
                description="Parameter file to use for camera tf",
            ),
            DeclareLaunchArgument(
                "zed_launch_args_file",
                default_value=DEFAULT_PARAMS_FILES["zed_launch"],
                description="File containing launch arguments for zed camera launch",
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
