import numpy as np
import rclpy

from hunav_msgs.msg import Agents, Agent
from rclpy.node import Node

from zed_interfaces.msg import ObjectsStamped
import tf2_geometry_msgs
import tf2_ros

class Zed2Nav(Node):
    def __init__(self):
        super().__init__('zed2nav_node')
        self._zed_subscriber = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self._zed_callback,
            10
        )
        self._human_state_publisher = self.create_publisher(
            Agents,
            '/human_states',
            10
        )


    def _zed_callback(self, msg):

        people_msg = Agents()
        people_msg.header.stamp = msg.header.stamp
        people_msg.header.frame_id = "map"

        id = 0
        for object in msg.objects:
            if object.label == 'Person':
                people_msg.agents.append(
                    self._new_agent(id=id, obj=object, 
                                    obj_frame=msg.header.frame_id))
                id += 1

        self._human_state_publisher.publish(people_msg)

        # for agent in msg.agents:
        #     self.get_logger().info(f'Agent ID: {agent.id}, Position: {agent.position}, Velocity: {agent.velocity}')
        # self.get_logger().info('Received agents data')

    def _new_agent(self, id, obj, obj_frame, target_frame='map'):
        '''
        Get pose from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
        Return a new agent: agent (Agent)
        '''
        agent = Agent()
        agent.id = id

        # Checking transformation frame
        frame_trans = self._check_transform(target_frame, obj_frame)

        # Getting the position of agent
        transformed_position = tf2_geometry_msgs.do_transform_point(
                obj.position, frame_trans)
        agent.position.position = transformed_position

        #TODO: Getting the orientation of agent

        return agent
    
    def _check_transform(self, target_frame, obj_frame):
        '''
        Inner function to check the transformation
        '''
        try:
            frame_trans=self.tf_buffer.lookup_transform(
                target_frame, obj_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"Transform check failed: {str(e)}")

        return frame_trans