from typing import List, Union

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Vector3
from people_msgs.msg import People, Person
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_geometry_msgs import do_transform_point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

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
            self.declare_parameter("goals", "[]").get_parameter_value().string_value
        )
        self.goals = str_to_list_of_np(self.goals)

        ### Publishers and subscribers
        if self.use_simulator:
            self._subscriber = self.create_subscription(
                People, "/people", self._position_callback, 10
            )

            self._human_goals_timer = self.create_timer(1.0, self._goal_callback)

            self._human_goals_publisher = self.create_publisher(
                MarkerArray, "/human_goals", 10
            )
        else:
            from zed_interfaces.msg import ObjectsStamped

            self._subscriber = self.create_subscription(
                ObjectsStamped,
                "/zed/zed_node/obj_det/objects",
                self._position_callback,
                10,
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._human_positions_world_publisher = self.create_publisher(
                People, "/human_positions_world", 10
            )

        self._human_positions_publisher = self.create_publisher(
            MarkerArray, "/human_positions", 10
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

    def _position_callback(self, msg):
        # Human positions
        human_positions_markers = MarkerArray()

        if self.use_simulator:
            num_ppl = len(msg.people)
            for i in range(num_ppl):
                x = msg.people[i].position.x
                y = msg.people[i].position.y
                color_rbga = self.human_colors[i % self.num_human_colors]
                human_positions_markers.markers.append(
                    create_marker(
                        [x, y, i],
                        marker_type="cylinder",
                        marker_namespace="human",
                        color_rgba=color_rbga,
                        z=0.75,
                        height=1.5,
                    )
                )
        else:  # Real world
            people_msg = People()
            people_msg.header.stamp = msg.header.stamp
            people_msg.header.frame_id = "map"

            for object in msg.objects:
                if object.label == "Person":
                    p1 = PointStamped()
                    p1.header = msg.header
                    p1.header.stamp = rclpy.time.Time().to_msg()

                    position = object.position
                    p1.point.x = float(position[0])
                    p1.point.y = float(position[1])
                    p1.point.z = float(position[2])

                    try:
                        p2 = self._tf_buffer.transform(p1, "map")
                    except TransformException as e:
                        self.get_logger().warn(f"Could not transform pose: {e}")
                        return

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

                    human_positions_markers.markers.append(
                        create_marker(
                            [x, y, object.label_id],
                            marker_type="cylinder",
                            marker_namespace="human",
                            color_rgba=color_rbga,
                            text=marker_text,
                        )
                    )
            self._human_positions_world_publisher.publish(people_msg)

        # Publish
        self._human_positions_publisher.publish(human_positions_markers)

    def _goal_callback(self):
        """
        Publishes ground-truth goals for all humans (only in simulation)
        """
        human_goals_markers = MarkerArray()

        for i in range(len(self.goals)):
            human_goals_markers = create_marker_array(
                self.goals[i] + self.goal_offsets[i],
                marker_type="cube",
                marker_namespace=f"goals_{i}",
                color_rgba=self.human_colors[i % self.num_human_colors],
                markers=human_goals_markers,
            )

        self._human_goals_publisher.publish(human_goals_markers)


def create_marker_array(
    xyis: np.ndarray,
    marker_type: str,
    marker_namespace: str,
    color_rgba: Union[List, np.ndarray],
    markers: MarkerArray = None,
):
    """
    xyis: list or array of (x,y) positions and ID's.
          Shape should be (,3) when listing IDs
          Shape should be (,2) when not listing IDs
              (in which case IDs will be the row number)
    """
    assert xyis.ndim == 2, "xyis must be 2D!"
    assert xyis.shape[1] == 2 or xyis.shape[1] == 3

    if xyis.shape[1] == 2:
        xyis = np.vstack((xyis.T, np.arange(xyis.shape[0]))).T

    if markers is None:
        markers = MarkerArray()

    for i in range(xyis.shape[0]):
        markers.markers.append(
            create_marker(
                xyis[i, :],
                marker_type=marker_type,
                marker_namespace=marker_namespace,
                color_rgba=color_rgba,
            )
        )

    return markers


def create_marker(
    xyi,
    marker_type="cube",
    marker_namespace="human",
    color_rgba=[0.1, 0.0, 1.0, 1.0],
    z=0.2,
    height=0.2,
    text: str = "",
):
    """
    Create a marker for visualization
    Possible types: 'cube', 'sphere', 'cylinder'
    """
    marker = Marker()

    # Position and ID
    x = xyi[0]
    y = xyi[1]
    id = int(xyi[2])

    # Type
    if marker_type == "cube":
        marker_type = marker.CUBE

    elif marker_type == "sphere":
        marker_type = marker.SPHERE

    elif marker_type == "cylinder":
        marker_type = marker.CYLINDER

    else:
        raise ValueError

    # Color
    r, g, b, a = color_rgba
    color = ColorRGBA(r=r, g=g, b=b, a=a)

    # Populate marker attributes
    marker.header.frame_id = "/map"
    marker.ns = marker_namespace
    marker.type = marker_type
    marker.action = marker.ADD
    marker.id = id
    marker.pose.position = Point(x=x, y=y, z=z)
    marker.scale = Vector3(x=0.2, y=0.2, z=height)
    marker.color = color
    marker.text = text

    return marker


def main(args=None):
    rclpy.init(args=args)
    node = PeopleVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
