import rclpy
from hunavis.transform_to_pose import TransformToPose
from rclpy.parameter import Parameter


class RobotPosePublisher(TransformToPose):
    """
    Publishes the robot pose based on the base_link to map transform to /robot_pose
    (by default). Thin wrapper around TransformToPose.
    """

    def __init__(self):
        super().__init__("robot_pose_publisher")

        self.set_parameters(
            [
                Parameter("source_frame_id", Parameter.Type.STRING, "base_link"),
                Parameter("target_frame_id", Parameter.Type.STRING, "map"),
                Parameter("pose_topic", Parameter.Type.STRING, "/robot_pose"),
            ]
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
