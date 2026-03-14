import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import argparse


class CmdVelJoyNode(Node):
    def __init__(self, debug):
        super().__init__("cmd_vel_joy_node")
        self.get_logger().info("cmd_vel_joy_node has been started.")
        self.debug = debug

        self.cmd_vel_joy_subscriber = self.create_subscription(
            TwistStamped, "/cmd_vel_joy", self.cmd_vel_joy_callback, 10
        )
        # pub topics
        self.cmd_vel_joy_pub = self.create_publisher(Twist, "/cmd_vel_joy_uros", 10)

    def cmd_vel_joy_callback(self, msg):
        # print(f"{msg.twist.linear.x:.2f}, {msg.twist.angular.z:.2f}")
        linear_x = float(msg.twist.linear.x)
        angular_z = float(msg.twist.angular.z)
        x, y = self.twist_mux_xz_to_uros_xy(linear_x, angular_z)
        if self.debug:
            self.get_logger().info(
                f"\nLinear x:{linear_x:.2f}, Angular z:{angular_z:.2f} => ({x:.2f}, {y:.2f})"
            )
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = x if x > 0 else 0.01  # 움직이지 않을 정도의 신호
        cmd_vel_msg.linear.y = y if y > 0 else 0.01  # 움직이지 않을 정도의 신호
        self.cmd_vel_joy_pub.publish(cmd_vel_msg)

    def twist_mux_xz_to_uros_xy(self, linear_x, angular_z):
        # uros_x = linear_x - 0.25 * angular_z
        # uros_y = linear_x + 0.25 * angular_z
        uros_x = linear_x - 0.5 * angular_z
        uros_y = linear_x + 0.5 * angular_z
        return (uros_x, uros_y)

    def uros_xy_to_twist_mux_xz(self, uros_x, uros_y):
        angular_z = (uros_y - uros_x) * 2
        # linear_x = uros_y - 0.25 * angular_z
        linear_x = uros_y - 0.50 * angular_z
        return (linear_x, angular_z)


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )

    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    node = CmdVelJoyNode(**args)
    rclpy.spin(node)  # Spin the main thread for ROS 2 callbacks
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
