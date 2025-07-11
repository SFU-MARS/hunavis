import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import TransformException
from rcl_interfaces.msg import SetParametersResult

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__("robot_pose_publisher")

        # Parameters
        self.declare_parameter("update_rate", 60.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_pose_topic", "/robot_pose")

        self.update_rate = (
            self.get_parameter("update_rate").get_parameter_value().double_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.robot_pose_topic = (
            self.get_parameter("robot_pose_topic").get_parameter_value().string_value
        )        

        # Setup callback for dynamically changing parameters
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Publisher for the robot pose in map frame
        self.pose_pub = self.create_publisher(PoseStamped, self.robot_pose_topic, 10)

        # Timer to periodically call the callback to publish the pose
        self.create_timer(1.0 / self.update_rate, self.timer_callback)

        # Create a tf2 buffer and listener for transforming to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        # Periodically publish the robot's pose in the specified frame
        try:
            # Lookup the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, "base_link", rclpy.time.Time()
            )

            # Create a PoseStamped message for the robot's pose in map frame
            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = self.frame_id

            # The transform contains the robot's pose in the map frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z

            # Set the orientation in the map frame (only yaw matters in 2D)
            pose.pose.orientation = transform.transform.rotation

            # Publish the pose in the map frame
            self.pose_pub.publish(pose)

        except TransformException as e:
            self.get_logger().warn(f"Could not transform pose: {e}")


    def parameter_callback(self, params):
        for param in params:
            if param.name == "frame_id" and param.type_ == param.Type.STRING:
                self.get_logger().info(f"frame_id updated: {self.frame_id} -> {param.value}")
                self.frame_id = param.value
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
