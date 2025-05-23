import math
import select
import sys
import termios
import threading
import tty
from typing import List

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


# Helper function to convert Euler angles (roll, pitch, yaw) to a quaternion
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class TFKeyboardPublisher(Node):
    """
    A ROS 2 node that broadcasts a TF transform and allows interactive control
    via keyboard input.

    Parameters:
    -----------
    - default_transform (list[float]): [x, y, z, roll, pitch, yaw] in meters/radians.
    - parent_frame_id (str): Parent frame ID (default: "map").
    - child_frame_id (str): Child frame ID (default: "base_link").
    - handedness (str): "right" (default) or "left" — left-handed inverts Y axis.
    - position_step (float): Step size in meters (default: 0.01).
    - rotation_step (float): Step size in degrees (default: 1.0).
    """

    def __init__(self) -> None:
        super().__init__("tf_keyboard_publisher")

        # Declare all parameters (initial values from YAML)
        self.declare_parameter("default_transform", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("parent_frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("handedness", "right")
        self.declare_parameter("position_step", 0.01)  # meters
        self.declare_parameter("rotation_step", 1.0)  # degrees

        # Get initial values from parameters
        self.default_transform = self.get_parameter("default_transform").value
        self.parent_frame = self.get_parameter("parent_frame_id").value
        self.child_frame = self.get_parameter("child_frame_id").value
        self.handedness = self.get_parameter("handedness").value
        self.position_step = self.get_parameter("position_step").value
        self.rotation_step_deg = self.get_parameter("rotation_step").value
        self.rotation_step = math.radians(self.rotation_step_deg)

        self.set_transform_from_list(self.default_transform)
        self.history = []

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)

        # Services
        self.create_service(Empty, "reset_transform", self.reset_callback)

        self.get_logger().info("TF keyboard node started.")
        self.print_help()

        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    def set_transform_from_list(self, vals: List[float]) -> None:
        """Set the current transform from a 6D list."""
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = vals

    def get_transform_as_list(self) -> List[float]:
        """Return the current transform as a 6D list."""
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]

    def save_history(self) -> None:
        """Save current transform to undo history."""
        self.history.append(self.get_transform_as_list())

    def publish_transform(self) -> None:
        """Broadcast the current transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        x, y, z = self.x, self.y, self.z
        if self.handedness == "left":
            y = -y

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

    def reset_callback(
        self, request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        """Reset transform to the default."""
        self.save_history()
        self.set_transform_from_list(self.default_transform)
        self.get_logger().info("Transform reset to default.")
        return response

    def keyboard_listener(self) -> None:
        """Listen for keyboard input to update the transform in real time."""

        def get_key() -> str:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            key = sys.stdin.read(1) if rlist else ""
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

        global settings
        settings = termios.tcgetattr(sys.stdin)

        move_bindings = {
            "q": (1, 0, 0),
            "a": (-1, 0, 0),
            "w": (0, 1, 0),
            "s": (0, -1, 0),
            "e": (0, 0, 1),
            "d": (0, 0, -1),
        }

        rotate_bindings = {
            "u": (1, 0, 0),
            "j": (-1, 0, 0),
            "i": (0, 1, 0),
            "k": (0, -1, 0),
            "o": (0, 0, 1),
            "l": (0, 0, -1),
        }

        try:
            while rclpy.ok():
                key = get_key()
                if key in move_bindings:
                    self.save_history()
                    dx, dy, dz = move_bindings[key]
                    self.x += dx * self.position_step
                    self.y += dy * self.position_step
                    self.z += dz * self.position_step
                    self.print_current_transform()
                elif key in rotate_bindings:
                    self.save_history()
                    dr, dp, dyaw = rotate_bindings[key]
                    self.roll += dr * self.rotation_step
                    self.pitch += dp * self.rotation_step
                    self.yaw += dyaw * self.rotation_step
                    self.print_current_transform()
                elif key == "r":
                    self.save_history()
                    self.set_transform_from_list(self.default_transform)
                    self.get_logger().info("Reset via keyboard.")
                    self.print_current_transform()
                elif key == "z":
                    if self.history:
                        self.set_transform_from_list(self.history.pop())
                        self.get_logger().info("Undo last change.")
                        self.print_current_transform()
                    else:
                        self.get_logger().info("No undo history.")
                elif key == "h":
                    self.handedness = "left" if self.handedness == "right" else "right"
                    self.get_logger().info(f"Switched handedness to: {self.handedness}")
                elif key == "=":
                    self.position_step *= 1.5
                    self.get_logger().info(
                        f"Position step increased: {self.position_step:.4f} m"
                    )
                elif key == "-":
                    self.position_step /= 1.5
                    self.get_logger().info(
                        f"Position step decreased: {self.position_step:.4f} m"
                    )
                elif key == "]":
                    self.rotation_step_deg *= 1.5
                    self.rotation_step = math.radians(self.rotation_step_deg)
                    self.get_logger().info(
                        f"Rotation step increased: {self.rotation_step_deg:.2f}°"
                    )
                elif key == "[":
                    self.rotation_step_deg /= 1.5
                    self.rotation_step = math.radians(self.rotation_step_deg)
                    self.get_logger().info(
                        f"Rotation step decreased: {self.rotation_step_deg:.2f}°"
                    )
                elif key == "/":
                    self.print_help()
                elif key == "\x03":  # CTRL+C
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def print_current_transform(self) -> None:
        """Print the current transform in a YAML-compatible format."""
        current_transform = self.get_transform_as_list()
        transform_yaml = (
            f"# Current transform (copy this to your YAML file to set as default)\n"
            f"default_transform: {current_transform}\n"
            f"parent_frame_id: {self.parent_frame}\n"
            f"child_frame_id: {self.child_frame}\n"
            f"handedness: {self.handedness}\n"
        )
        self.get_logger().info(transform_yaml)

    def print_help(self) -> None:
        """Print concise keyboard usage instructions."""
        help_msg = (
            "\nKeyboard Controls:\n"
            + "Translation:  Q/A → +X/-X  W/S → +Y/-Y  E/D → +Z/-Z\n"
            + "Rotation:     U/J → +Roll/-Roll  I/K → +Pitch/-Pitch  O/L → +Yaw/-Yaw\n"
            + "Commands:     R → Reset transform  Z → Undo last change\n"
            + "              H → Toggle handedness  =/- → Change position step\n"
            + "              ]/[ → Change rotation step\n"
            + "Press / for help again.\n"
        )
        self.get_logger().info(help_msg)


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = TFKeyboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
