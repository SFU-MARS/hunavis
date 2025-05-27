"""
Starts human detection using a Zed2 camera. It launches
- rviz
- zed_wrapper_launch (starts Zed2 camera if use_simulator:=False)
- people_visualizer_node (shows detected humans as markers in rviz)

Arguments
 - use_simulator: 
    - If True, zed_wrapper_launch would not run. Instead, the scenario specified in 
      scenario_params_file will be simulated using hunavsim.
    - If False, zed_camera.launch.py (from zed_wrapper) would run, using launch
      arguments specified in the yaml file specified by zed_launch_args_file.
    - Note: For fixed cameras (e.g. detached from a robot), the tf_keyboard_publisher
            node from this repo can be run to adjust the map->camera tf in real time.
            Example parameters of that node are found in params/zed_common.yaml, 
            which is also the zed node parameters file.

- scenario_params_file: (ignored if use_simulator:=False)
    - Parameter file defining the simulation scenario. 
      Defaults to params/hunavsim.yaml

- zed_launch_args_file: (ignored if use_simulator:=True)
    - File containing the launch arguments to zed_camera.launch.py, including another
      parameter file that specifies the parameters of the zed node.
      Defaults to params/zed_launch_args.yaml

- run_rviz:
    - Whether to run rviz

- rviz_file:
    - File containing rviz settings
"""

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

DEFAULT_PARAMS_FILES = {
    "scenario": os.path.join(
        get_package_share_directory("hunavis"),
        "params",
        SCENARIO_PARAMS_FILE,
    ),
    "zed_launch": os.path.join(
        get_package_share_directory("hunavis"), "params", "zed_launch_args.yaml"
    ),
    "rviz": os.path.join(
        get_package_share_directory("hunavis"), "rviz", "default_sim_view.yaml"
    ),    
}


def launch_setup(context, *args, **kwargs):
    use_simulator = LaunchConfiguration("use_simulator")
    scenario_params_file = LaunchConfiguration("scenario_params_file")
    zed_launch_args_file = LaunchConfiguration("zed_launch_args_file")
    run_rviz = LaunchConfiguration("run_rviz")
    rviz_file = LaunchConfiguration("rviz_file")

    scenario_params_file_val = scenario_params_file.perform(context)
    zed_launch_args_file_val = zed_launch_args_file.perform(context)
    rviz_file_val = rviz_file.perform(context)

    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d" + rviz_file_val],
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
        people_visualizer_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulator",
                default_value="True",
                description="Whether to use simulator.",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "scenario_params_file",
                default_value=DEFAULT_PARAMS_FILES["scenario"],
                description=(
                    "Params file specifying scenario if use_simulator:=True."
                ),
            ),
            DeclareLaunchArgument(
                "zed_launch_args_file",
                default_value=DEFAULT_PARAMS_FILES["zed_launch"],
                description=(
                    "Launch args for zed camera if use_simulator:=False."
                ),
            ),
            DeclareLaunchArgument(
                "run_rviz",
                default_value="True",
                description="Whether to use rviz.",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "rviz_file",
                default_value=DEFAULT_PARAMS_FILES["rviz"],
                description=(
                    "File containing rviz settings."
                ),
            ),            
            OpaqueFunction(function=launch_setup),
        ]
    )
