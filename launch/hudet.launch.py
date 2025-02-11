import json
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution

from hunavis.utils import goal_from_params

"""
Human Detection Launch File
"""

# Replace with yaml file containing parameters for hunavsim environment
DEFAULT_PARAMS_FILE = os.path.join(
    get_package_share_directory("hunavis"), "params", "params.yaml"
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
    )  # Values are specified manually, and would need to change if the camera pose changes wrt map

    # Load list of human goals from the simulation parameters
    humans_goals_str = goal_from_params(params_file_val)

    # Load list of human goals from the simulation parameters
    with open(params_file_val, "r") as f:
        config = yaml.safe_load(f)
        params = config["hunav_loader"]["ros__parameters"]
        humans_initial_pose = []
        humans_goals = []
        for agent in params["agents"]:
            agent_goals = []
            for goal_key in params[agent]["goals"]:
                goal = params[agent][goal_key]
                agent_goals.append([goal["x"], goal["y"]])

            initial_pose = params[agent]["init_pose"]
            initial_pose = [initial_pose["x"], initial_pose["y"]]

            humans_initial_pose.append(initial_pose)
            humans_goals.append(agent_goals)

    # String representing list of goals as 2D np arrays
    humans_goals_str = json.dumps(humans_goals)

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

