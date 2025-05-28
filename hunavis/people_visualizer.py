from typing import List, Union

import numpy as np
import rclpy

from hunav_msgs.msg import Agents
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from hunavis.utils import str_to_list_of_np


class PeopleVisualizer(Node):
    '''
    Receiving people msg from topic: /human_states (Agents)
    Publish to topic: /human_markers (MarkerArray)
    If simulation:
        From: self.goals (list [[x,y,h]...]) 
        publish to topic: /goal_markers (MarkerArray)
    '''
    def __init__(self):
        super().__init__("people_visualizer_node")

        # Declare if using simulator
        self.use_simulator = (
            self.declare_parameter("use_simulator", False)
            .get_parameter_value()
            .bool_value
        )
        # Initialize the goal
        self.goals = (
            self.declare_parameter("goals", "[]")
            .get_parameter_value()
            .string_value
        )
        self.goals = str_to_list_of_np(self.goals)

        # Subscriber
        self._subscriber = self.create_subscription(
            Agents, "/human_states", self._human_callback, 10
        )
        # Publisher of /human_markers
        self._human_markers_publisher = self.create_publisher(
            MarkerArray, "/human_markers", 10
        )

        # Save the color information to yaml file
        self.colors = [
            [0.1, 0.1, 1.0, 0.7],
            [0.1, 1.0, 0.1, 0.7],
            [0.1, 1.0, 1.0, 0.7],
            [1.0, 0.1, 0.1, 0.7],
            [1.0, 0.1, 1.0, 0.7],
            [1.0, 1.0, 0.1, 0.7],
            [1.0, 1.0, 1.0, 0.7],
        ]
        self.num_colors = len(self.colors)

        # If use simulator, potential goals need to be visualized
        if self.use_simulator:
            # Random offsets to clearly show goals common to multiple people
            self.num_goal_offsets = 10
            self.goal_offsets = -0.05 + 0.1 * np.random.random(self.num_goal_offsets)

            self._human_goals_timer = self.create_timer(1.0, self._goals_callback)
            self._goals_publisher = self.create_publisher(
                MarkerArray, "/goal_markers", 10
            )
        

    def _human_callback(self, msg):
        '''
        Get human pose from topic: /human_states (Agents)
        Process and publish to: /human_markers (MarkerArray) (Marker[]) 
        '''
        human_markers = MarkerArray()

        for agent in msg.agents:
            pose = np.array([agent.position.position.x, 
                             agent.position.position.y, 
                             agent.yaw])
            
            color_rgba = self.colors[agent.id%self.num_colors]

            # TODO: try
            # marker = Marker()
            human_markers.append(
                self._create_marker(
                    id = agent.id,
                    marker_pose = pose,
                    marker_type = 0, #ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3
                    marker_namespace = "human",
                    color_rgba = color_rgba,
                    z = 0.75,
                    height = 1.5,
                )
            )
        self._human_markers_publisher.publish(human_markers)


    def _goals_callback(self):
        '''
        Publishes ground-truth goals for all humans (only in simulation)
        '''
        goal_markers = MarkerArray()

        for i in range(len(self.goals)):
            goal_markers.append(
                self._create_marker(
                    id = i,
                    marker_pose = self.goals[i] + self.goal_offsets[i%self.num_goal_offsets],
                    marker_namespace = f"goals_{i}",
                    color_rgba = self.colors[i%self.num_colors],
                    z = 1.25,
                    height = 1.25
                )
            )

        self._goals_publisher.publish(goal_markers)


    def _create_marker(self, id, marker_pose, marker_type=1, 
                      marker_namespace="human", color_rgba=[0.1, 0.0, 1.0, 1.0], 
                      z=0.2, height=0.2, text: str = ""):
        '''
        Inner function to create a Marker object for human visualization.
        Input:
        - marker_pose (np.array): [x,y,yaw] or [x,y,None]
        - marker_type (int): ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3
        Output:
        - marker (Marker)
        '''
        marker = Marker()

        # Header
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "/map"
    
        # getting marker position
        marker.pose.position.x = marker_pose[0]
        marker.pose.position.y = marker_pose[1]
        marker.pose.position.z = z

        # getting marker orientation
        if marker_pose[2] is not None:
            qx, qy, qz, qw = quaternion_from_euler(roll=0, pitch=0, 
                                                   yaw=marker_pose[2])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw

        # Marker specifics
        marker.ns = marker_namespace
        marker.id = id
        marker.action = marker.ADD
        marker.text = text
        marker.type = marker_type

        # marker color
        r, g, b, a = color_rgba
        marker.color = ColorRGBA(r=r, g=g, b=b, a=a)

        # Marker scale
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = height

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = PeopleVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
