import os
import yaml
from pyproj import Transformer

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand


class DemoNode(Node):
    def __init__(self):
        super().__init__("test_coords_collect_node")
        self.get_logger().info("Initializing test_coords_collect_node")
        self.latitude = 36.5665
        self.longitude = 127.9780
        self.yaw = 0.0

        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32652", always_xy=True
        )
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )

        self.yaml_file_path = os.path.join(os.path.dirname(__file__), "wps.yaml")
        self.coords_list = []

        self.action_server = ActionServer(
            self,
            SimpleCommand,
            "test_coords_collect",
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

        self.get_logger().info(
            f"GPS: lat {self.latitude:.6f}, lon {self.longitude:.6f}"
        )
        self.coords_list.append(
            {"latitude": self.latitude, "longitude": self.longitude}
        )
        with open(self.yaml_file_path, "w") as wps_file:
            yaml.dump(self.coords_list, wps_file, sort_keys=False)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info("test_coords_collect action canceled")
            result.success = False
            return result

        feedback.progress = 0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        self.get_logger().info("test_coords_collect action completed successfully")
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

    node = DemoNode()
    try:
        asyncio.run(main_async(node))
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
