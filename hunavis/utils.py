import ast
import json
from typing import List

import numpy as np
import yaml


def str_to_list_of_np(s: str) -> List[np.ndarray]:
    """
    Converts a numpy array in string format into a list of numpy arrays
    """
    ls = ast.literal_eval(s)
    return [np.array(arr) for arr in ls]


def goal_from_params(params_file_val):
    """
    Reads a parameter file and obtains the list of human goals as a string
    """
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

    return humans_goals_str
