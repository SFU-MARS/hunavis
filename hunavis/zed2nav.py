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
    '''
    Receive msg from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
    Extract human and publish to topic: /human_states (Agents)
    '''
    def __init__(self):
        super().__init__('zed2nav_node')

        self._zed_subscriber = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self._zed_callback,
            10
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._human_state_publisher = self.create_publisher(
            Agents,
            '/human_states',
            10
        )


    def _zed_callback(self, msg):
        '''
        Get pose from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
        Return people_msg; publish to /human_states  (Agents)
        - human_states (Agents)
            - header (std_msgs/Header)
            - agents (hunav_msgs/Agent[])
        '''

        human_states = Agents()
        human_states.header.stamp = msg.header.stamp
        human_states.header.frame_id = 'map'

        human_states.agents = []
        for object in msg.objects:
            if object.label == 'Person':
                agent = self._new_agent(id=object.label_id, obj=object, 
                                    obj_frame=msg.header.frame_id,
                                    target_frame=human_states.header.frame_id)
                human_states.agents.append(agent)

                self.get_logger().debug(f'Getting Agent: {agent.id}')
        
        self._human_state_publisher.publish(human_states)
        self.get_logger().debug('Publish to /human_states')


    def _new_agent(self, id, obj, obj_frame, target_frame):
        '''
        Get pose from topic: /zed/zed_node/obj_det/objects (ObjectsStamped)
        Return a new agent: 
        - agent (Agent)
            - id (int32)
            - position (geometry_msgs/Pose)
                -position (Point)
                -orientation (Quaternion) Not implemented
            - yaw (float32) Not implemented
        '''
        agent = Agent()
        agent.id = id
        
        # Getting the position of the agent
        transformed_position = self._get_transformed_position(target_frame, 
                                                              obj_frame, obj.position)
        agent.position.position = transformed_position.point

        agent.linear_vel, agent.yaw = self._get_transfromed_velocity(target_frame, 
                                                                     obj_frame, obj.velocity)

        return agent

    def _get_transfromed_velocity(self, target_frame, obj_frame, velocity):
        '''
        Inner function to get the velocity transformed to the target_frame:
        - linear_vel (float): the linear velocity of agent
        - yaw (float): the heading angle of agent
        '''
        frame_trans = self._check_transform(target_frame, obj_frame)
        if frame_trans is None:
            linear_vel = Vector3Stamped()
            return linear_vel
        
        linear_vel = Vector3Stamped()
        linear_vel.vector.x = float(velocity[0])
        linear_vel.vector.y = float(velocity[1])
        linear_vel.vector.z = float(velocity[2])

        transformed_velocity = tf2_geometry_msgs.do_transform_vector3(
                linear_vel, frame_trans)
        # transformed_velocity = obj_point
        
        x_vel = transformed_velocity.vector.x
        y_vel = transformed_velocity.vector.y

        linear_vel = np.sqrt(x_vel ** 2 + y_vel ** 2)
        yaw = np.arctan2(y_vel, x_vel)

        return linear_vel, yaw
    

    def _get_transformed_position(self, target_frame, obj_frame, position):
        '''
        Inner function to get the position transformed to target_frame:
        - transformed_position (PointStamped)
            -point (Point)
                -x (float)
                -y (float)
                -z (float)
        '''
        # Checking transformation frame
        frame_trans = self._check_transform(target_frame, obj_frame)
        if frame_trans is None:
            obj_point = PointStamped()
            return obj_point

        # Getting the position of agent
        obj_point = PointStamped()
        obj_point.point.x = float(position[0])
        obj_point.point.y = float(position[1])
        obj_point.point.z = float(position[2])

        transformed_position = tf2_geometry_msgs.do_transform_point(
                obj_point, frame_trans)
        
        return transformed_position


    def _check_transform(self, target_frame, obj_frame):
        '''
        Inner function to check the transformation based on the frames
        '''

        frame_trans = None
        try:
            frame_trans=self._tf_buffer.lookup_transform(
                target_frame=target_frame, 
                source_frame=obj_frame, 
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"Transform check failed: {str(e)}")

        return frame_trans


def main(args=None):
    rclpy.init(args=args)
    node = Zed2Nav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()