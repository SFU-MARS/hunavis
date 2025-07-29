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

from geometry_msgs.msg import PoseWithCovarianceStamped


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


def euler_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple:
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).
    """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


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
        self.locked = True

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)

        # Subscribers and Services
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback, 10
        )

        self.create_service(Empty, "reset_transform", self.reset_callback)

        self.get_logger().info("TF keyboard node started.")
        self.print_help()
        self.warn_if_locked()

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

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Update transform from an initial pose message."""
        if self.warn_if_locked():
            return

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = self.z  # unchanged

        # Convert quaternion to Euler angles (yaw only)
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        self.yaw = yaw
        self.get_logger().info(
            f"Updated pose from /initialpose: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.2f}°"
        )

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
                if key == "c":
                    self.locked = not self.locked
                    status = "locked" if self.locked else "unlocked"
                    self.get_logger().info(f"Transform is now {status}.")
                elif key in move_bindings:
                    if self.warn_if_locked():
                        continue
                    self.save_history()
                    dx, dy, dz = move_bindings[key]
                    self.x += dx * self.position_step
                    self.y += dy * self.position_step
                    self.z += dz * self.position_step
                    self.print_current_transform()
                elif key in rotate_bindings:
                    if self.warn_if_locked():
                        continue
                    self.save_history()
                    dr, dp, dyaw = rotate_bindings[key]
                    self.roll += dr * self.rotation_step
                    self.pitch += dp * self.rotation_step
                    self.yaw += dyaw * self.rotation_step
                    self.print_current_transform()
                elif key == "r":
                    if self.warn_if_locked():
                        continue
                    self.save_history()
                    self.set_transform_from_list(self.default_transform)
                    self.get_logger().info("Reset via keyboard.")
                    self.print_current_transform()
                elif key == "z":
                    if self.warn_if_locked():
                        continue
                    if self.history:
                        self.set_transform_from_list(self.history.pop())
                        self.get_logger().info("Undo last change.")
                        self.print_current_transform()
                    else:
                        self.get_logger().info("No undo history.")
                elif key == "h":
                    if self.warn_if_locked():
                        continue
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
        transform_yaml = (
            f"# Current transform (copy this to your YAML file to set as default)\n"
            f"current_transform: {self.get_transform_as_list()}\n"
            f"parent_frame_id: {self.parent_frame}\n"
            f"child_frame_id: {self.child_frame}\n"
            f"handedness: {self.handedness}\n"
        )
        self.get_logger().info(transform_yaml)

    def warn_if_locked(self) -> bool:
        """Log a warning if locked. Returns True if locked, False otherwise."""
        GREEN = "\033[92m"
        RESET = "\033[93m"
        if self.locked:
            self.get_logger().warn(
                f"\nTransform is currently LOCKED.\n"
                + f"Press {GREEN}C{RESET} to toggle lock, {GREEN}/{RESET} for help.\n"
            )

            return True
        return False

    def print_help(self) -> None:
        """Print concise and formatted keyboard usage instructions with colors."""
        CYAN = "\033[96m"
        YELLOW = "\033[93m"
        GREEN = "\033[92m"
        RESET = "\033[0m"

        help_msg = (
            f"\n{CYAN}==================== Keyboard Controls ===================={RESET}\n"
            f"{YELLOW}Translation:{RESET}   {GREEN}Q/A{RESET} → +X/-X     {GREEN}W/S{RESET} → +Y/-Y     {GREEN}E/D{RESET} → +Z/-Z\n"
            f"{YELLOW}Rotation:{RESET}      {GREEN}U/J{RESET} → +Roll/-Roll   {GREEN}I/K{RESET} → +Pitch/-Pitch   {GREEN}O/L{RESET} → +Yaw/-Yaw\n"
            f"{YELLOW}Transform Ops:{RESET} {GREEN}R{RESET} → Reset    {GREEN}Z{RESET} → Undo    {GREEN}C{RESET} → Toggle lock\n"
            f"{YELLOW}Misc:{RESET}          {GREEN}H{RESET} → Toggle handedness\n"
            f"                {GREEN}= / -{RESET} → Increase/decrease position step\n"
            f"                {GREEN}] / [{RESET} → Increase/decrease rotation step\n"
            f"{CYAN}==========================================================={RESET}\n"
            f"Press {GREEN}/{RESET} to reprint this help message.\n"
            + f"Current transform: {self.get_transform_as_list()}\n"
        )
        self.get_logger().info(help_msg)


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    node = TFKeyboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
