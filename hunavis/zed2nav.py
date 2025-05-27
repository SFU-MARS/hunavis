import numpy as np
import rclpy

from geometry_msgs.msg import Point, PointStamped
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
        '''

        human_states = Agents()
        human_states.header.stamp = msg.header.stamp
        human_states.header.frame_id = 'map'

        # id = 0
        human_states.agents = []
        for object in msg.objects:
            if object.label == 'Person':
                agent = self._new_agent(id=object.label_id, obj=object, 
                                    obj_frame=msg.header.frame_id)
                human_states.agents.append(agent)
                # id += 1
                self.get_logger().info(
                        f'Agent ID: {agent.id}, Position: {agent.position}, Velocity: {object.velocity}')
                

        self._human_state_publisher.publish(human_states)


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
        obj_point = PointStamped()
        obj_point.point.x = float(obj.position[0])
        obj_point.point.y = float(obj.position[1])
        obj_point.point.z = float(obj.position[2])

        transformed_position = tf2_geometry_msgs.do_transform_point(
                obj_point, frame_trans)
        agent.position.position = Point(x=transformed_position.point.x,
                                        y=transformed_position.point.y,
                                        z=transformed_position.point.z)
        
        #TODO: Try
        agent.position.position = transformed_position.point

        #TODO: Getting the orientation of agent

        return agent
    
    def _check_transform(self, target_frame="map", obj_frame="zed_camera_center"):
        '''
        Inner function to check the transformation
        '''
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