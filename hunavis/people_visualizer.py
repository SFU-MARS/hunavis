from typing import List, Union

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Vector3
# from people_msgs.msg import People, Person
from hunav_msgs.msg import Agents
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_geometry_msgs import do_transform_point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from zed_interfaces.msg import ObjectsStamped

from hunavis.utils import str_to_list_of_np


class PeopleVisualizer(Node):
    def __init__(self):
        super().__init__("people_visualizer_node")

        ### Parameters
        self.use_simulator = (
            self.declare_parameter("use_simulator", False)
            .get_parameter_value()
            .bool_value
        )
        self.goals = (
            self.declare_parameter("goals", "[]")
            .get_parameter_value()
            .string_value
        )
        self.goals = str_to_list_of_np(self.goals)

        ### Publishers and subscribers
        if self.use_simulator:
            self._subscriber = self.create_subscription(
                Agents, "/human_states", self._human_callback, 10
            )

            # TODO: Unsure about the goals
            self._human_goals_timer = self.create_timer(1.0, self._goals_callback)
            self._goals_publisher = self.create_publisher(
                MarkerArray, "/goal_markers", 10
            )

        else:
            self._subscriber = self.create_subscription(
                ObjectsStamped,
                "/zed/zed_node/obj_det/objects",
                self._human_callback,
                10,
            )
            
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            # TODO: check if conflict
            self._human_positions_world_publisher = self.create_publisher(
                Agents, "/human_states", 10
            )

        self._human_markers_publisher = self.create_publisher(
            MarkerArray, "/human_markers", 10
        )
        ### Member attributes
        self.human_colors = [
            [0.1, 0.1, 1.0, 0.7],
            [0.1, 1.0, 0.1, 0.7],
            [0.1, 1.0, 1.0, 0.7],
            [1.0, 0.1, 0.1, 0.7],
            [1.0, 0.1, 1.0, 0.7],
            [1.0, 1.0, 0.1, 0.7],
            [1.0, 1.0, 1.0, 0.7],
        ]
        self.num_human_colors = len(self.human_colors)

        # Random offsets to clearly show goals common to multiple people
        self.goal_offsets = -0.05 + 0.1 * np.random.random(len(self.goals))


    def _sim_human_state_callback(self, msg):
        """
        Get human pose from topic: /human_states (Agents)
        Process and save the marker information to: self.human_markers (MarkerArray)  
        """
        num_people = len(msg.agents)
        marker = Marker()

        for i in range(num_people):
            pose = np.array([msg.agents[i].position.position.x, 
                             msg.agents[i].position.position.y, 
                             msg.agents[i].yaw])

            color_rbga = self.human_colors[i % self.num_human_colors]

            self.human_markers.append(
                self._create_marker(
                    id = i,
                    marker_pose = pose,
                    marker_type = marker.ARROW,
                    marker_namespace = "human",
                    color_rgba = color_rbga,
                    z = 0.75,
                    height = 1.5,
                )
            )

    
    def _real_human_state_callback(self, msg):
        """
        Get human pose from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
        Process and save the marker information to: 
                                            - self.human_markers (MarkerArray)  
                                            - self.human_states (Agents)
        """

        for i, object in enumerate(msg.objects):
            if object.label == "Person":
                position = object.position
                pose = np.array([position[0], position[1], object.label_id])

                color_rbga = self.human_colors[object.label_id % self.num_human_colors]

                # Append agent for prediction

                # Append marker for visualization
                self.human_markers.append(
                    self._create_marker(
                        id=i,
                        marker_pose=pose,
                        marker_type="cylinder",
                        marker_namespace="human",
                        color_rgba=color_rbga,
                        text=object.action_state,
                    )
                )

        self._human_positions_publisher.publish(human_positions_markers)



    def _human_callback(self, msg):
        # Human positions
        self.human_markers = MarkerArray()

        if self.use_simulator:
            self._sim_human_state_callback(msg)
        else:  # Real world
            self._real_human_state_callback(msg)

            people_msg = Agents()
            people_msg.header.stamp = msg.header.stamp
            people_msg.header.frame_id = "map"

            for object in msg.objects:
            # TODO: if this is the step that slows down the system
                if object.label == "Person":
                    p1 = PointStamped()
                    p1.header = msg.header

                    position = object.position
                    p1.point.x = float(position[0])
                    p1.point.y = float(position[1])
                    p1.point.z = float(position[2])

                    p2 = self._tf_buffer.transform(p1, "map")
                    x = p2.point.x
                    y = p2.point.y

                    person = Person()
                    person.position = Point(x=x, y=y)
                    people_msg.people.append(person)

                    color_rbga = self.human_colors[
                        object.label_id % self.num_human_colors
                    ]

                    if object.action_state == 1:
                        marker_text = "moving"
                    else:
                        marker_text = "stationary"

                    self.human_markers.markers.append(
                        self._create_marker(
                            [x, y, object.label_id],
                            marker_type="cylinder",
                            marker_namespace="human",
                            color_rgba=color_rbga,
                            text=marker_text,
                        )
                    )
            self._human_positions_world_publisher.publish(people_msg)

        # Publish
        self._human_positions_publisher.publish(self.human_markers)

    # TODO: Unsure about the goals, why they are written in array?
    def _goals_callback(self):
        """
        Publishes ground-truth goals for all humans (only in simulation)
        """
        goal_markers = MarkerArray()
        marker = Marker()

        for i in range(len(self.goals)):
            goal_markers = self._create_marker_array(
                id = i,
                marker_pose = self.goals[i] + self.goal_offsets[i],
                marker_type=marker.CUBE,
                marker_namespace=f"goals_{i}",
                color_rgba=self.human_colors[i % self.num_human_colors],
                markers=goal_markers,
            )

        self._goals_publisher.publish(goal_markers)

    # TODO: Unsure about the goals, why using array instead of single marker?
    def _create_marker_array(self, id,
        marker_pose: np.ndarray,
        marker_type: str,
        marker_namespace: str,
        color_rgba: Union[List, np.ndarray],
        markers: MarkerArray = None,
    ):
        """
        marker_pose: list or array of (x,y) positions.
            Shape should be (,3) when listing IDs
            Shape should be (,2) when not listing IDs
                (in which case IDs will be the row number)
        """
        assert marker_pose.ndim == 2, "xyis must be 2D!"
        assert marker_pose.shape[1] == 2 or marker_pose.shape[1] == 3

        if marker_pose.shape[1] == 2:
            marker_pose = np.vstack((marker_pose.T, np.arange(marker_pose.shape[0]))).T

        if markers is None:
            markers = MarkerArray()

        for i in range(marker_pose.shape[0]):
            markers.markers.append(
                self._create_marker(
                    id=id,
                    marker_pose=marker_pose[i, :],
                    marker_type=marker_type,
                    marker_namespace=marker_namespace,
                    color_rgba=color_rgba,
                )
            )

        return markers


    def _create_marker(self, id, marker_pose, marker_type="default", 
                      marker_namespace="human", color_rgba=[0.1, 0.0, 1.0, 1.0], 
                      z=0.2, height=0.2, text: str = ""):
        """
        Inner function to create a Marker object for human visualization.
        """
        marker = Marker()

        # Header
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "/map"

        # Marker specifics
        marker.ns = marker_namespace
        marker.id = id
        marker.action = marker.ADD
        marker.text = text

        # Marker type
        if marker_type == "default":
            marker_type = marker.CUBE
        marker.type = marker_type
    
        # Marker pose
        marker.pose.position.x = marker_pose[0]
        marker.pose.position.y = marker_pose[1]
        marker.pose.position.z = z

        if marker_pose.shape[0] == 3:
            qx, qy, qz, qw = quaternion_from_euler(roll=0, pitch=0, 
                                                   yaw=marker_pose[2])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw

        # Marker color
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
