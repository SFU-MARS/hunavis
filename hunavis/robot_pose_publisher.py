import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import TransformException


class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__("robot_pose_publisher")

        # Parameters
        self.update_rate = 60.0

        # Publisher for the robot pose in map frame
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)

        # Timer to periodically call the callback to publish the pose
        self.create_timer(1.0 / self.update_rate, self.timer_callback)

        # Create a tf2 buffer and listener for transforming to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def timer_callback(self):
        # Periodically publish the robot's pose in the map frame
        self.publish_robot_pose()

    def publish_robot_pose(self):
        try:
            # Lookup the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )

            # Create a PoseStamped message for the robot's pose in map frame
            map_pose = PoseStamped()
            map_pose.header.stamp = transform.header.stamp
            map_pose.header.frame_id = "map"

            # The transform contains the robot's pose in the map frame
            map_pose.pose.position.x = transform.transform.translation.x
            map_pose.pose.position.y = transform.transform.translation.y
            map_pose.pose.position.z = transform.transform.translation.z

            # Set the orientation in the map frame (only yaw matters in 2D)
            map_pose.pose.orientation = transform.transform.rotation

            # Publish the pose in the map frame
            self.pose_pub.publish(map_pose)

        except TransformException as e:
            self.get_logger().warn(f"Could not transform pose: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
