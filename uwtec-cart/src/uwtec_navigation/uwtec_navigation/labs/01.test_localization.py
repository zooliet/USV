import argparse
from datetime import datetime
from pyproj import Transformer

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse


# from sensor_msgs.msg import NavSatFix
from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand
from uwtec_navigation.utils import signed_angle


class DemoNode(Node):
    def __init__(self, interval=1.0, offset=10, debug=False):
        super().__init__("test_localization_node")
        self.interval = interval
        self.offset = offset  # offset between GPS heading and Gyro heading, in degrees
        self.debug = debug

        self.latitude = 36.5665
        self.longitude = 127.9780
        self.yaw = 0.0

        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32652", always_xy=True
        )
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )

        self.action_server = ActionServer(
            self,
            SimpleCommand,
            "test_localization",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )
        self.yaw = -msg.heading % 360  # convert to CCW positive
        # self.heading = east_facing_ccw_angle(self.heading)

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        feedback = SimpleCommand.Feedback()
        result = SimpleCommand.Result()
        rate = self.create_rate(int(1.0 / self.interval))  # Hz
        while rclpy.ok():  # or while True:
            current_time = datetime.now().strftime("%H:%M:%S")
            self.get_logger().info(f"Timer tick at {current_time}")
            self.get_logger().info(
                f"GPS: lat {self.latitude:.6f}, lon {self.longitude:.6f}"
            )
            self.get_logger().info(f"UTM: x {self.utm_x:.2f}, y {self.utm_y:.2f}")
            self.get_logger().info(
                f"Gyro Yaw and Offset: {self.yaw:.2f}, {self.offset:.2f}"
            )
            current_heading = (self.yaw - self.offset) % 360
            signed_heading = signed_angle(current_heading)
            self.get_logger().info(
                f"Heading(ccw): {current_heading:.2f}, Singed Heading: {signed_heading:.2f}"
            )
            self.get_logger().info("--------------------------------------")

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("test_localization action canceled")
                result.success = False
                return result

            feedback.progress = 0  # Update progress
            goal_handle.publish_feedback(feedback)

            try:
                rate.sleep()
            except Exception as _:
                # Handle case where ROS context shuts down
                result.success = False
                return result

        goal_handle.succeed()
        self.get_logger().info("test_localization action completed successfully")
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
        "-i", "--interval", type=float, default=1.0, help="timer interval: 1.0*"
    )
    ap.add_argument("--offset", type=int, default=10, help="Gryo heading offset: 10*")
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
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
