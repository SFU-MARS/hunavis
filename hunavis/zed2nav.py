import rclpy
import numpy as np

from hunav_msgs.msg import Agents, Agent
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Point, PointStamped
from hunav_msgs.msg import Agents, Agent
from geometry_msgs.msg import PointStamped, Vector3Stamped
from zed_interfaces.msg import ObjectsStamped


class Zed2Nav(Node):
    """
    Receive msg from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
    Extract human and publish to topic: /human_states (Agents)
    """

    def __init__(self):
        super().__init__("zed2nav_node")

        self._zed_subscriber = self.create_subscription(
            ObjectsStamped, "/zed/zed_node/obj_det/objects", self._zed_callback, 10
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._human_state_publisher = self.create_publisher(Agents, "/human_states", 10)

    def _zed_callback(self, msg: ObjectsStamped):
        """
        Get pose from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
        Return people_msg; publish to /human_states  (Agents)
        - human_states (Agents)
            - header (std_msgs/Header)
            - agents (hunav_msgs/Agent[])
        """
        # Don't publish anything without knowing the world coordinates (due to
        # transform being unavailable)
        human_frame_id = "map"

        frame_trans = self._check_transform(human_frame_id, msg.header.frame_id)

        if frame_trans is None:
            return

        human_states = Agents()
        human_states.header.stamp = msg.header.stamp
        human_states.header.frame_id = human_frame_id

        human_states.agents = []

        for object in msg.objects:
            if object.label.casefold() == "person":
                agent = self._new_agent(
                    id=object.label_id,
                    obj=object,
                    frame_trans=frame_trans,
                )
                human_states.agents.append(agent)

                self.get_logger().debug(f"Getting Agent: {agent.id}")

        self._human_state_publisher.publish(human_states)
        self.get_logger().debug("Publish to /human_states")

    def _new_agent(self, id: int, obj: ObjectsStamped, frame_trans):
        """
        Get pose from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
        Return a new agent:
        - agent (Agent)
            - id (int32)
            - position (geometry_msgs/Pose)
                -position (Point)
                -orientation (Quaternion) Not implemented
            - yaw (float32) Not implemented
        """
        agent = Agent()
        agent.id = id

        transformed_position = self._get_transformed_position(frame_trans, obj.position)
        agent.position.position = transformed_position.point

        x_vel, y_vel = self._get_transformed_velocity(
            frame_trans, obj.velocity
        )
        agent.velocity.linear.x, agent.velocity.linear.y = x_vel, y_vel
        agent.linear_vel = np.sqrt(x_vel**2 + y_vel**2)
        agent.yaw = np.arctan2(y_vel, x_vel)

        return agent

    def _get_transformed_velocity(self, frame_trans, velocity):
        """
        Inner function to get the velocity transformed to the target_frame:
        - linear_vel (float): the linear velocity of agent
        - yaw (float): the heading angle of agent
        """
        linear_vel = Vector3Stamped()

        linear_vel.vector.x = float(velocity[0])
        linear_vel.vector.y = float(velocity[1])
        linear_vel.vector.z = float(velocity[2])

        transformed_velocity = tf2_geometry_msgs.do_transform_vector3(
            linear_vel, frame_trans
        )

        x_vel = transformed_velocity.vector.x
        y_vel = transformed_velocity.vector.y



        return x_vel, y_vel

    def _get_transformed_position(self, frame_trans, position):
        """
        Inner function to get the position transformed to target_frame:
        - transformed_position (PointStamped)
            -point (Point)
                -x (float)
                -y (float)
                -z (float)
        """
        # Getting the position of agent
        obj_point = PointStamped()
        obj_point.point.x = float(position[0])
        obj_point.point.y = float(position[1])
        obj_point.point.z = float(position[2])

        transformed_position = tf2_geometry_msgs.do_transform_point(
            obj_point, frame_trans
        )

        return transformed_position

    def _check_transform(self, target_frame, obj_frame):
        """
        Inner function to check the transformation based on the frames
        """

        frame_trans = None
        try:
            frame_trans = self._tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=obj_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as e:
            self.get_logger().warn(f"Transform check failed: {str(e)}")

        return frame_trans


def main(args=None):
    rclpy.init(args=args)
    node = Zed2Nav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
