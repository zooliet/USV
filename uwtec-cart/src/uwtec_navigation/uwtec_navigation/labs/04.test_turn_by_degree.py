import argparse
import os
import yaml
from datetime import datetime
from pyproj import Transformer
from ament_index_python.packages import get_package_share_directory

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

from geometry_msgs.msg import Twist
from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand
from uwtec_navigation.utils import (
    # utm_bearing,
    # signed_angle,
    # calc_offset,
    calc_heading_by_offset,
    calc_goal_heading,
    rotate_to_go,
)


class DemoNode(Node):
    def __init__(self, interval=0.1, accuracy=3.0, angular_speed=0.5, debug=False):
        super().__init__("test_turn_by_degree_node")
        self.interval = interval
        self.accuracy = accuracy
        self.angular_speed = angular_speed
        self.debug = debug

        self.latitude = 36.5665
        self.longitude = 127.9780
        self.yaw = 0.0
        self.gps_quality = 0
        self.num_sats = 0

        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32652", always_xy=True
        )
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )
        self.heading_yaml_path = os.path.join(
            get_package_share_directory("uwtec_navigation"), "config", "heading.yaml"
        )
        with open(self.heading_yaml_path, "r") as heading_file:
            heading_data = yaml.safe_load(heading_file)
            self.offset = heading_data.get("offset", 0.0)

        self.action_server = ActionServer(
            self,
            SimpleCommand,
            "test_turn_by_degree",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.cmd_vel_nav_pub = self.create_publisher(
            Twist,
            # "/cmd_vel_nav",
            "/cmd_vel",  # 임시
            1,
        )

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

    def go_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.01
        twist_msg.linear.y = 0.01
        self.cmd_vel_nav_pub.publish(twist_msg)

    def stop_drive(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.01
        twist_msg.linear.y = 0.01
        self.cmd_vel_nav_pub.publish(twist_msg)

    def turn_around(self, angle):
        twist_msg = Twist()
        if angle > 0:  # ccw turn, turn left
            twist_msg.linear.x = 0.01
            twist_msg.linear.y = self.angular_speed
        else:  # cw turn, turn right
            twist_msg.linear.x = self.angular_speed
            twist_msg.linear.y = 0.01
        self.cmd_vel_nav_pub.publish(twist_msg)

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.angular_speed
        twist_msg.linear.y = 0.01
        self.cmd_vel_nav_pub.publish(twist_msg)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )
        self.yaw = -msg.heading % 360  # convert to CCW positive
        self.gps_quality = msg.gps_quality
        self.num_sats = msg.num_sats
        # self.heading = east_facing_ccw_angle(self.heading)

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = SimpleCommand.Feedback()
        result = SimpleCommand.Result()

        # initialize local variables
        theta = int(goal_handle.request.cmd)
        heading = calc_heading_by_offset(self.yaw, self.offset)
        goal_heading = calc_goal_heading(heading, by=theta)

        # Reset ticks for each execution
        ticks_for_5_secs = int(5.0 / self.interval)
        ticks_for_1_sec = int(1.0 / self.interval)

        rate = self.create_rate(int(1.0 / self.interval))  # Hz
        while rclpy.ok():  # or while True:
            current_heading = calc_heading_by_offset(self.yaw, self.offset)
            remaining_angle = rotate_to_go(current_heading, goal_heading)
            if abs(remaining_angle) < 3:  # within 3 degrees of the goal
                self.stop_drive()
                break
            else:
                self.turn_around(remaining_angle)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("test_turn_bo_degree action canceled")
                self.stop_drive()
                result.success = False
                return result

            feedback.progress = 0  # Update progress
            goal_handle.publish_feedback(feedback)

            ## tick updates
            ticks_for_5_secs = (
                ticks_for_5_secs - 1
                if ticks_for_5_secs > 1
                else int(5.0 / self.interval)
            )
            ticks_for_1_sec = (
                ticks_for_1_sec - 1
                if ticks_for_1_sec > 1.0
                else int(1.0 / self.interval)
            )
            if ticks_for_1_sec % 10 == 0 and self.debug:
                current_time = datetime.now().strftime("%H:%M:%S")
                self.get_logger().info(f"{remaining_angle:.2f} at {current_time}")

            try:
                rate.sleep()
            except Exception as _:
                # Handle case where ROS context shuts down
                result.success = False
                return result

        goal_handle.succeed()
        self.get_logger().info("test_turn_bo_degree action completed successfully")
        result.success = True
        return result


async def ros_loop(nodes):
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        while executor.context.ok():
            # while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)
            await asyncio.sleep(0.001)
    finally:
        for node in nodes:
            executor.remove_node(node)
        executor.shutdown()


async def main_async(*nodes):
    ros_tasks = asyncio.create_task(ros_loop(list(nodes)))
    await ros_tasks


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-i", "--interval", type=float, default=0.1, help="timer interval: 0.1*"
    )
    ap.add_argument("--accuracy", type=float, default=3.0, help="target accuracy: 3.0*")
    ap.add_argument(
        "--angular-speed",
        type=float,
        default=0.5,
        help="Angular speed in [0-1] (default: 0.5*)",
    )

    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False*)"
    )
    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    node = DemoNode(**args)
    try:
        asyncio.run(main_async(node))
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
